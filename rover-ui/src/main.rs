mod data;
use crate::data::data::server;
use crate::data::AppState;
use actix_files::Files;
use actix_web::middleware::Logger;
use actix_web::web::{Bytes, Data};
use actix_web::{get, rt, web, App, Error, HttpRequest, HttpResponse, HttpServer, Responder};
use actix_ws::AggregatedMessage;
use env_logger::Env;
use futures_util::StreamExt;
use std::io::Read;
use std::sync::{Arc, Mutex};
use tokio::sync::broadcast::{Receiver, Sender};
use tokio::sync::{broadcast, RwLock};
use tracing::debug;

#[get("/rgb_feed")]
async fn rgb_feed(
    mut app_state: Data<AppState>,
    req: HttpRequest,
    stream: web::Payload,
) -> Result<HttpResponse, Error> {
    debug!("RGB feed");
    handle_video_stream(app_state.rgb_frame.subscribe(), req, stream)
}

#[get("/depth_feed")]
async fn depth_feed(
    mut app_state: Data<AppState>,
    req: HttpRequest,
    stream: web::Payload,
) -> Result<HttpResponse, Error> {
    debug!("Depth feed");
    handle_video_stream(app_state.rgb_frame.subscribe(), req, stream)
}

fn handle_video_stream(
    mut recv: Receiver<Bytes>,
    req: HttpRequest,
    stream: web::Payload,
) -> Result<HttpResponse, Error> {
    let (res, mut session, stream) = actix_ws::handle(&req, stream)?;

   
    // start task but don't wait for it

    rt::spawn(async move {
        loop {
            let Ok(frame) = recv.recv().await else {
                continue;
            };

            if let Err(e) = session.binary(frame.clone()).await {
                tracing::debug!("Closing Stream");
                break;
            }
        }
    });

    // respond immediately with response connected to WS session
    Ok(res)
}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    env_logger::init_from_env(Env::default().default_filter_or("debug"));

    let (depth_frame, _) = broadcast::channel(1);

    let (rgb_frame, _) = broadcast::channel(1);
    let app_data = Data::new(AppState {
        rgb_frame: rgb_frame.clone(),
        depth_frame: depth_frame.clone(),
    });
    rt::spawn(async move {
        server(rgb_frame, depth_frame).await;
    });

    HttpServer::new(move || {
        let app = App::new();

        app.wrap(Logger::default())
            .app_data(app_data.clone())
            .service(rgb_feed)
            .service(depth_feed)
            .service(Files::new("/", "ui/dist").index_file("index.html"))
    })
    .bind("0.0.0.0:8081")?
    .run()
    .await
}
