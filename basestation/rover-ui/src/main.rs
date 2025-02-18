mod data;
use std::net::IpAddr;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use crate::data::data::server;
use actix_files::Files;
use actix_web::middleware::Logger;
use actix_web::web::{Bytes, Data, Json};
use actix_web::{get, post, rt, web, App, Error, HttpRequest, HttpResponse, HttpServer, Responder};
use actix_ws::AggregatedMessage;
use futures_util::StreamExt;
use prost::Message as _;
use rover_msgs::rover::Message;
use serde::Deserialize;
use tokio::select;
use tokio::sync::mpsc::{self, Sender};
use tokio::sync::{oneshot, watch, Mutex};
use tracing::debug;
use webrtc::mdns::conn;

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
    let (tx,rx) = oneshot::channel::<String>();
    app_state.send((body.local_session_description.clone(),tx)).await.unwrap();

    let response = rx.await.unwrap();

    Ok(HttpResponse::Ok().body(response))
}



#[get("/message_stream")]
async fn message_stream(
    sender: Data<tokio::sync::mpsc::Sender<Message>>,
    sender_broadcast: Data<tokio::sync::broadcast::Sender<Message>>,
    conn_recv : Data<watch::Receiver<Option<IpAddr>>>,
    req: HttpRequest, stream: web::Payload
) -> Result<HttpResponse, Error> {
    let (res, mut session, stream) = actix_ws::handle(&req, stream)?;

    let mut stream = stream
        .aggregate_continuations()
        // aggregate continuation frames up to 1MiB
        .max_continuation_size(2_usize.pow(20));
    let mut conn_recv = conn_recv.as_ref().clone();

    // start task but don't wait for it
    rt::spawn(async move {
        // receive messages from websocket
        debug!("Starting websocket from client");
        let connected = conn_recv.borrow().clone();
        let connected = connected.as_ref().map(|ip| ip.to_string()).unwrap_or("None".to_string());
        session.text(connected).await.unwrap(); 
        let mut recv = sender_broadcast.subscribe();
        loop{


            select!{
                connected = conn_recv.changed() => {
                    match connected {
                        Ok(_) => {
                            let connected = conn_recv.borrow().clone();
                            let connected = connected.as_ref().map(|ip| ip.to_string()).unwrap_or("None".to_string());
                            session.text(connected).await.unwrap();
                        }
                        _ => {

                        }
                    }
                }
                msg = stream.next() => {
                    match msg {
                        Some(Ok(AggregatedMessage::Binary(bin))) => {
                            let msg:Message = Message::decode(bin).unwrap();
                            sender.send(msg).await.unwrap()
                        }
                        _ => {
                            break;
                        }
                    }
                }
                msg = recv.recv() => {
                    match msg {
                        Ok(msg) => {
                            let mut buf = Vec::new();
                            buf.reserve(msg.encoded_len());
                            // Unwrap is safe, since we have reserved sufficient capacity in the vector.
                            msg.encode(&mut buf).unwrap();                    
                            
                            session.binary(buf).await.unwrap();
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
    std::env::set_var("RUST_LOG", "rover_ui=debug");
    
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let (sender,recv) = mpsc::channel::<(String,oneshot::Sender<String>)>(10);

    rt::spawn(async move {
        server(recv).await.unwrap();
    });

    let (rover_sender,rover_recv) = mpsc::channel::<Message>(20);

    let (message_send,message_recv) = tokio::sync::broadcast::channel::<Message>(10);


    let (send,recv) = tokio::sync::watch::channel::<Option<IpAddr>>(None);
    let msg1 = message_send.clone();
    rt::spawn(async move {
        data::rover::launch_rover_link(rover_recv,msg1,send).await.unwrap();
    });

    let sender:DescriptorExhange = sender;


    HttpServer::new(move || {
        let app = App::new();

        app.wrap(Logger::default())
        .app_data(Data::new(message_send.clone()))
            .app_data(Data::new(sender.clone()))
            .app_data(Data::new(rover_sender.clone()))
            .app_data(Data::new(recv.clone()))
            .service(start_stream)
            .service(message_stream)
            .service(Files::new("/", "ui/dist").index_file("index.html"))
    })
    .disable_signals()
    .bind("0.0.0.0:8081")?
    .run()
    .await
}
