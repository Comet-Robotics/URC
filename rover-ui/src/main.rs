mod data;
use crate::data::data::server;
use crate::data::AppState;
use actix_files::Files;
use actix_web::middleware::Logger;
use actix_web::web::{Bytes, Data};
use actix_web::{get, post, rt, web, App, Error, HttpRequest, HttpResponse, HttpServer, Responder};
use actix_ws::AggregatedMessage;
use env_logger::Env;
use futures_util::StreamExt;
use serde::Deserialize;
use tokio::sync::mpsc::{self, Sender};
use tokio::sync::oneshot;
use std::io::Read;
use std::sync::{Arc, Mutex};
use tracing::debug;

type DescriptorExhange = Sender<(String, oneshot::Sender<String>)>;
#[derive(Deserialize)]
struct Exchange{
    #[serde(rename = "localSessionDescription")]
    local_session_description: String,
}

#[post("/start_stream")]
async fn rgb_feed(
    mut app_state: Data<DescriptorExhange>,
    body: web::Json<Exchange>
) -> Result<HttpResponse, Error> {
    debug!("RGB feed");
    let (tx,rx) = oneshot::channel::<String>();
    app_state.send((body.local_session_description.clone(),tx)).await.unwrap();

    let response = rx.await.unwrap();


    Ok(HttpResponse::Ok().body(response))


}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    env_logger::init_from_env(Env::default().default_filter_or("debug"));
    
    let (sender,recv) = mpsc::channel::<(String,oneshot::Sender<String>)>(10);

    rt::spawn(async move {
        server(recv).await.unwrap();
    });

    let sender:DescriptorExhange = sender;

    HttpServer::new(move || {
        let app = App::new();

        app.wrap(Logger::default())
            .app_data(Data::new(sender.clone()))
            .service(rgb_feed)
            .service(Files::new("/", "ui/dist").index_file("index.html"))
    })
    .bind("0.0.0.0:8081")?
    .run()
    .await
}
