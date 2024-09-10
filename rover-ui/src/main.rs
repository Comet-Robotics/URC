use std::io::Read;
use std::sync::{Arc, Mutex};
use actix_files::Files;
use actix_web::{get, rt, web, App, Error, HttpRequest, HttpResponse, HttpServer, Responder};
use actix_web::middleware::Logger;
use actix_web::web::{Bytes, Data};
use actix_ws::AggregatedMessage;
use env_logger::Env;
use futures_util::StreamExt;
use tokio::sync::{broadcast, RwLock};
use tokio::sync::broadcast::{Receiver, Sender};
#[cfg(feature = "real")]
use crate::real_data::AppState;

#[cfg(feature = "real")]
#[get("/rgb_feed")]
async fn rgb_feed(mut app_state:Data<AppState>,req: HttpRequest, stream: web::Payload) -> Result<HttpResponse, Error> {
    handle_video_stream(app_state.rgb_frame.subscribe(),req,stream)
}

#[cfg(feature = "real")]
#[get("/depth_feed")]
async fn depth_feed(mut app_state:Data<AppState>,req: HttpRequest, stream: web::Payload) -> Result<HttpResponse, Error> {
    handle_video_stream(app_state.depth_frame.subscribe(),req,stream)

}


fn handle_video_stream(mut recv: Receiver<Bytes>,req: HttpRequest, stream: web::Payload)-> Result<HttpResponse, Error>{
    let (res, mut session, stream) = actix_ws::handle(&req, stream)?;

    let mut stream = stream
        .aggregate_continuations()
        // aggregate continuation frames up to 1MiB
        .max_continuation_size(2_usize.pow(20));

    // start task but don't wait for it


    rt::spawn(async move {
        loop{
            let Ok(frame) = recv.recv().await else{
                continue
            };
            
            
            if let Err(e) = session.binary(frame.clone()).await{
                tracing::debug!("Closing Stream");
                break;
            }
            
        }

    });

    // respond immediately with response connected to WS session
    Ok(res)}



#[cfg(feature = "real")]
mod real_data {
    use std::ffi::c_void;
    use std::io::Write;
    use std::pin::Pin;
    use std::process::Stdio;
    use tokio::process::{Command};
    use actix_web::web;

    use futures_util::stream::StreamExt;
    use opencv::core::{Mat, Mat2b, MatTraitConst, MatTraitConstManual, ToInputArray, Vector, VectorToVec};
    use opencv::imgcodecs::imencode;
    use opencv::types::VectorOfu8;
    use opencv::{core, imgcodecs};

    use opencv::imgproc;

    use r2r::sensor_msgs::msg::Image;
    use r2r::{Context, Node, QosProfile, Timer};
    use std::sync::{mpsc, Arc, Mutex};
    use std::task::Poll;
    use std::thread;
    use actix_web::web::{Bytes, Data};
    use futures_util::{Stream, TryStreamExt};
    use chrono::{DateTime,Utc};
    use tokio::io::AsyncWriteExt;
    use tokio::select;
    use tokio::sync::RwLock;
    use tokio::sync::broadcast::Sender;

    fn image_to_opencv(image: &Image) -> Result<Mat,opencv::Error> {

        // Determine the type of the image based on encoding
        let mat_type = match image.encoding.as_str() {
            "rgb8" => core::CV_8UC3,   // 8-bit unsigned, 3 channels (RGB)
            "bgr8" => core::CV_8UC3,   // 8-bit unsigned, 3 channels (BGR)
            "mono8" => core::CV_8UC1,  // 8-bit unsigned, 1 channel (grayscale)
            "16UC1" => core::CV_16UC1,
            _ => {
                return Err(opencv::Error::new(
                    core::StsError,
                    format!("Unsupported encoding: {}", image.encoding),
                ));
            }
        };

        // Get a pointer to the raw image data
        let raw_data_ptr = image.data.as_ptr() as *mut c_void;

        // Determine the step size (the number of bytes per row)
        // For tightly packed rows, this is width * number of channels * size of each element (u8 = 1 byte)
        let step =match image.encoding.as_str(){
            "mono8" =>  (image.width * 1) as usize,
            "16UC1" =>  (image.width as usize * std::mem::size_of::<u16>()) ,
            _=>  (image.width * 3) as usize,
        };

        // SAFETY: We are using an unsafe function to construct the Mat with raw data.
        // We must ensure that the raw data pointer is valid for the lifetime of the Mat and that the dimensions match.
        let mat = unsafe {
            Mat::new_rows_cols_with_data_unsafe(
                image.height as i32,  // Height (rows)
                image.width as i32,   // Width (cols)
                mat_type,             // Type (based on encoding)
                raw_data_ptr,         // Pointer to raw data
                step,                 // Step size (bytes per row)
            ).unwrap()
        };

        if image.encoding == "16UC1" {
            // Convert 16-bit depth image to 8-bit for display purposes
            let mut normalized_depth = Mat::default();

            // Normalize the depth image to a range of 0-255
            let mut depth_8bit = Mat::default();
            mat.convert_to(&mut depth_8bit, core::CV_8UC1, 255.0 / 5000.0, 0.0).unwrap(); // Assumes depth range of 0-10 meters

            // Apply the COLORMAP_JET or any other color map to the normalized image
            let mut color_depth = Mat::default();
            imgproc::apply_color_map(&depth_8bit, &mut color_depth, imgproc::COLORMAP_JET).unwrap();

            return Ok(color_depth);
        }



        Ok(mat)
    }

    fn img_encode(mat: &Mat)->Bytes{
        let mut buf = Vector::<u8>::new();

        imencode(".jpg",mat,&mut buf,&Vector::new()).unwrap();


       Bytes::copy_from_slice(buf.as_slice())

    }

    pub struct AppState {
        pub rgb_frame:Sender<Bytes>,


        pub depth_frame: Sender<Bytes>,

    }

    pub async fn ros2_listeners( ctx: Context,rgb_send: Sender<Bytes>,depth_send:  Sender<Bytes>) {
        let mut node = Node::create(ctx, "rust_web_server", "").unwrap();
        tracing::info!("Subscribing to camera");
        let mut rgb_sub = node
            .subscribe::<Image>("/camera/camera/color/image_raw", QosProfile::default())
            .unwrap();

        let mut depth_sub = node
            .subscribe::<Image>("/camera/camera/depth/image_rect_raw", QosProfile::default())
            .unwrap();


        tracing::info!("Reading frames from camera");

        let task = thread::spawn(  move || loop{
            node.spin_once(std::time::Duration::from_millis(100));
        });

        
        loop {
            

            select! {   
                Some(frame) = rgb_sub.next() => {
                    tracing::debug!("New Frame");
                
                    if (rgb_send.receiver_count() ==0){
                        continue;
                    }                    
                    let mat = image_to_opencv(&frame).unwrap();
                    let rgb = img_encode(&mat);
                    rgb_send.send(rgb).unwrap();
                }
                
                Some(frame) = depth_sub.next() => {
                    if (depth_send.receiver_count() ==0){
                        continue;
                    }
                    let mat = image_to_opencv(&frame).unwrap();
                    
                    let depth = img_encode(&mat);
                    depth_send.send(depth).unwrap();
                }                
            }
        }

    }

}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    env_logger::init_from_env(Env::default().default_filter_or("debug"));
    let mut app_data= None;
    #[cfg(feature = "real")]
    {
        let ctx = r2r::Context::create().unwrap();

        let (depth_frame,_) = broadcast::channel(1); 
        
        let (rgb_frame,_) =broadcast::channel(1); 
        app_data = Some(Data::new(AppState{rgb_frame:rgb_frame.clone(),depth_frame:depth_frame.clone()}));
        rt::spawn(async move {
            real_data::ros2_listeners(ctx,rgb_frame,depth_frame).await;
        });
    }
    HttpServer::new(move || {
        let mut app = App::new();
        if let Some(data)= app_data.clone(){
            app = app.app_data(data);
        }
        app.wrap(Logger::default())
            .service(rgb_feed)
            .service(depth_feed)
            
            .service(Files::new("/", "ui/dist").index_file("index.html"))
    })
    .bind("0.0.0.0:8080")?
    .run()
    .await
}
