mod data;
use crate::data::data::server;
use actix_files::Files;
use actix_web::middleware::Logger;
use actix_web::web::{Bytes, Data, Json};
use actix_web::{get, post, rt, web, App, Error, HttpRequest, HttpResponse, HttpServer, Responder};
use actix_ws::AggregatedMessage;
use env_logger::Env;
use futures_util::StreamExt;
use rover_msgs::{Message, Twist};
use serde::Deserialize;
use tokio::select;
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
async fn start_stream(
    app_state: Data<DescriptorExhange>,
    body: web::Json<Exchange>
) -> Result<HttpResponse, Error> {
    debug!("RGB feed");
    let (tx,rx) = oneshot::channel::<String>();
    app_state.send((body.local_session_description.clone(),tx)).await.unwrap();

    let response = rx.await.unwrap();

    Ok(HttpResponse::Ok().body(response))
}


#[get("/message_stream")]
async fn message_stream(
    sender: Data<std::sync::mpsc::Sender<Message>>,
    sender_broadcast: Data<tokio::sync::broadcast::Sender<Message>>,
    req: HttpRequest, stream: web::Payload
) -> Result<HttpResponse, Error> {
    let (res, mut session, stream) = actix_ws::handle(&req, stream)?;

    let mut stream = stream
        .aggregate_continuations()
        // aggregate continuation frames up to 1MiB
        .max_continuation_size(2_usize.pow(20));

    // start task but don't wait for it
    rt::spawn(async move {
        // receive messages from websocket
        let mut recv = sender_broadcast.subscribe();
        loop{


            select!{
                msg = stream.next() => {
                    match msg {
                        Some(Ok(AggregatedMessage::Text(text))) => {
                            let msg:Message = serde_json::from_str(&text).unwrap();
                            sender.send(msg).unwrap();
                        }
                        _ => {
                            break;
                        }
                    }
                }
                msg = recv.recv() => {
                    match msg {
                        Ok(msg) => {
                            debug!("Sending msg to websocket");
                            let text = serde_json::to_string(&msg).unwrap();
                            session.text(text).await.unwrap();
                        }
                        _ => {}
                    }
                }
            }


        }
    });

    // respond immediately with response connected to WS session
    Ok(res)
}


#[actix_web::main]
async fn main() -> std::io::Result<()> {
    env_logger::init_from_env(Env::default().default_filter_or("debug"));
    
    let (sender,recv) = mpsc::channel::<(String,oneshot::Sender<String>)>(10);

    rt::spawn(async move {
        server(recv).await.unwrap();
    });

    let (rover_sender,rover_recv) = std::sync::mpsc::channel::<Message>();

    let (message_send,message_recv) = tokio::sync::broadcast::channel::<Message>(10);
    let msg1 = message_send.clone();
    rt::task::spawn_blocking(move ||{
        data::rover::launch_rover_link(rover_recv,msg1).unwrap();
    });

    let sender:DescriptorExhange = sender;



    HttpServer::new(move || {
        let app = App::new();

        app.wrap(Logger::default())
        .app_data(Data::new(message_send.clone()))
            .app_data(Data::new(sender.clone()))
            .app_data(Data::new(rover_sender.clone()))
            .service(start_stream)
            .service(message_stream)
            .service(Files::new("/", "ui/dist").index_file("index.html"))
    })
    .disable_signals()
    .bind("0.0.0.0:8081")?
    .run()
    .await
}
