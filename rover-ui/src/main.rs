use std::sync::{Arc, Mutex};
use actix_files::Files;
use actix_web::{get, web, App, HttpResponse, HttpServer, Responder};
use serde::Serialize;
#[cfg(feature = "real")]

use crate::real_data::AppState;

#[derive(Serialize)]
struct SensorData {
    temperature: f64,
    pressure: f64,
    humidity: f64,
}

#[cfg(not(feature = "real"))]
#[get("/sensor_data")]
async fn sensor_data() -> impl Responder {
    let data = SensorData {
        temperature: 25.3,
        pressure: 1013.0,
        humidity: 45.0,
    };
    HttpResponse::Ok().json(data)
}

#[cfg(feature = "real")]
#[get("/sensor_data")]
async fn sensor_data(data: web::Data<AppState>) -> impl Responder {
    let sensor_data = data.sensor_data.lock().unwrap();
    HttpResponse::Ok().json(&*sensor_data)
}

#[cfg(not(feature = "real"))]
#[get("/video_feed")]
async fn video_feed() -> impl Responder {
    HttpResponse::Ok()
        .content_type("image/jpeg")
        .body(include_bytes!("../mock_data/mock_frame.jpg").as_ref())
}

#[cfg(feature = "real")]
#[get("/video_feed")]
async fn video_feed(data: web::Data<AppState>) -> impl Responder {
    let camera_frame = data.camera_frame.lock().unwrap();
    if let Some(frame) = &*camera_frame {
        HttpResponse::Ok()
            .content_type("image/jpeg")
            .body(frame.clone())
    } else {
        HttpResponse::Ok().body("No frame available")
    }
}

#[cfg(feature = "real")]
mod real_data {
    use r2r::sensor_msgs::msg::Image;
    use r2r::{Context, Node};
    use std::sync::{Arc, Mutex};
    use actix_web::web;
    use opencv::gapi::Image;
    use crate::SensorData;

    pub struct AppState {
        pub sensor_data: Arc<Mutex<SensorData>>,
        pub camera_frame: Arc<Mutex<Option<Vec<u8>>>>,
    }

    pub async fn ros2_listeners(app_data: web::Data<AppState>, ctx: Context) {
        let mut node = Node::new(ctx, "rust_web_server").unwrap();
        let sensor_sub = node.subscribe::<SensorData>("/sensor/data", 10).unwrap();
        let camera_sub = node
            .subscribe::<Image>("/camera/color/image_raw", 10)
            .unwrap();

        loop {
            if let Ok(Some(msg)) = sensor_sub.recv().await {
                let mut data = app_data.sensor_data.lock().unwrap();
                data.temperature = msg.temperature;
                data.pressure = msg.pressure;
                data.humidity = msg.humidity;
            }
            if let Ok(Some(image_msg)) = camera_sub.recv().await {
                let mut frame = app_data.camera_frame.lock().unwrap();
                let jpeg_frame = decode_image_msg_to_jpeg(&image_msg);
                *frame = Some(jpeg_frame);
            }
        }
    }

    fn decode_image_msg_to_jpeg(image_msg: &Image) -> Vec<u8> {
        // Decoding logic
        opencv::im
    }
}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    

    HttpServer::new(move || {
        let mut  app = App::new();
        #[cfg(feature = "real")]
        {
        let ctx = r2r::Context::new().unwrap();

        let app_state = web::Data::new(crate::real_data::AppState {
            sensor_data: Arc::new(Mutex::new(SensorData {
                temperature: 0.0,
                pressure: 0.0,
                humidity: 0.0,
        })),
        camera_frame: Arc::new(Mutex::new(None)),
    });
    let app_state_clone = app_state.clone();
    actix_web::rt::spawn(async move {
        real_data::ros2_listeners(app_state_clone, ctx).await;
    });
    app = app.app_data(app_state);

        }
   
        app.service(sensor_data)
            .service(video_feed)
            .service(Files::new("/","ui/dist").index_file("index.html"))
    })
        .bind("0.0.0.0:8080")?
        .run()
        .await
}
