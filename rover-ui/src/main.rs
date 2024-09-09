#[cfg(feature = "real")]
use crate::real_data::AppState;
use actix_files::Files;
use actix_web::{get, web, App, HttpResponse, HttpServer, Responder};
use std::sync::{Arc, Mutex};
use actix_web::middleware::Logger;
use env_logger::Env;
use chrono::Utc;





#[cfg(feature = "real")]
mod real_data {
    use std::ffi::c_void;
    use std::io::Write;
    use std::pin::Pin;
    use std::process::Stdio;
    use tokio::process::{Command};
    use actix_web::web;

    use futures_util::stream::StreamExt;
    use opencv::core::{Mat, Mat2b, MatTraitConst, ToInputArray, Vector, VectorToVec};
    use opencv::imgcodecs::imencode;
    use opencv::types::VectorOfu8;
    use opencv::{core, imgcodecs};


    use r2r::sensor_msgs::msg::Image;
    use r2r::{Context, Node, QosProfile, Timer};
    use std::sync::{mpsc, Arc, Mutex};
    use std::task::Poll;
    use std::thread;
    use futures_util::{Stream, TryStreamExt};
    use chrono::{DateTime,Utc};
    use tokio::io::AsyncWriteExt;

    async fn send_frame_to_ffmpeg(ffmpeg_process: &mut tokio::process::Child, mat: &Mat) -> Result<(), Box<dyn std::error::Error>> {
        // Convert Mat to JPEG format
        let mut buf = Vector::<u8>::new();
        imencode(".jpg", mat, &mut buf, &Vector::new())?;

        // Send the JPEG data to FFmpeg via stdin
        let mut stdin = ffmpeg_process.stdin.take().unwrap();
        stdin.write_all(buf.as_slice()).await?;
        
        
        ffmpeg_process.stdin = Some(stdin);
        
        Ok(())
    }

    fn image_to_opencv(image: &Image) -> Result<Mat,opencv::Error> {

        // Determine the type of the image based on encoding
        let mat_type = match image.encoding.as_str() {
            "rgb8" => core::CV_8UC3,   // 8-bit unsigned, 3 channels (RGB)
            "bgr8" => core::CV_8UC3,   // 8-bit unsigned, 3 channels (BGR)
            "mono8" => core::CV_8UC1,  // 8-bit unsigned, 1 channel (grayscale)
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
        let channels = if image.encoding == "mono8" { 1 } else { 3 };
        let step = (image.width * channels) as usize;

        // SAFETY: We are using an unsafe function to construct the Mat with raw data.
        // We must ensure that the raw data pointer is valid for the lifetime of the Mat and that the dimensions match.
        let mat = unsafe {
            Mat::new_rows_cols_with_data_unsafe(
                image.height as i32,  // Height (rows)
                image.width as i32,   // Width (cols)
                mat_type,             // Type (based on encoding)
                raw_data_ptr,         // Pointer to raw data
                step,                 // Step size (bytes per row)
            )?
        };

        Ok(mat)
    }

    pub struct AppState {
        pub camera_frame: Arc<Mutex<Option<Vec<u8>>>>,
    }

    pub async fn ros2_listeners( ctx: Context) {
        let mut node = Node::create(ctx, "rust_web_server", "").unwrap();
        tracing::info!("Subscribing to camera");
        let mut camera_sub = node
            .subscribe::<Image>("/camera/camera/color/image_raw", QosProfile::default())
            .unwrap();


        tracing::info!("Reading frames from camera");

        let task = thread::spawn(  move || loop{
            node.spin_once(std::time::Duration::from_millis(100));
        });
        //WEB RTC
        let mut ffmpeg_process = Command::new("ffmpeg")
            .args(&[
                "-y",                    // Overwrite output files
                "-f", "image2pipe",       // Input format
                "-r", "15",               // Frame rate
                "-i", "-",                // Input comes from stdin
                "-vf", "scale=640:360",
                "-preset","ultrafast",
                "-b:v","500k",
                "-c:v", "libx264",        // Codec to encode video
                "-pix_fmt", "yuv420p",    // Pixel format
                "-hls_time", "1",         // HLS segment duration
                "-hls_list_size", "3",    // Number of segments to keep in the manifest
                "-hls_wrap", "10",        // Wrap segment names after this number
                "-start_number", "1",     // Starting segment number
                "-hls_flags","delete_segments+omit_endlist",
                "video-data/rgb-data.m3u8",            // Output HLS playlist file
            ])
            .stdin(Stdio::piped())        // Pipe for input
            .stdout(Stdio::null())        // Ignore stdout
            .spawn().unwrap();  



        loop {

            if let Some(image_msg) = camera_sub.next().await {
                let mat = image_to_opencv(&image_msg).unwrap();
                send_frame_to_ffmpeg(&mut ffmpeg_process,&mat).await.unwrap();
            
            }else{
                tracing::error!("No Frame Found");
            }
        }

    }

}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    env_logger::init_from_env(Env::default().default_filter_or("debug"));
    #[cfg(feature = "real")]
    {
        let ctx = r2r::Context::create().unwrap();

        actix_web::rt::spawn(async move {
            real_data::ros2_listeners(ctx).await;
        });
    }
    HttpServer::new(move || {
        let mut app = App::new();
      


        app.wrap(Logger::default())
            .service(Files::new("/video-data/","video-data"))
            .service(Files::new("/", "ui/dist").index_file("index.html"))
    })
    .bind("0.0.0.0:8080")?
    .run()
    .await
}
