use actix_web::web::{self, Buf, BufMut, BytesMut};
use rtp::packet::Packet;
use std::ffi::c_void;
use std::io::Write;
use std::pin::Pin;
use std::process::Stdio;
use tokio::net::UdpSocket;
use tokio::process::Command;
use tracing::{debug, warn};
use webrtc_util::marshal::Unmarshal;
use webrtc_util::Buffer;

use futures_util::stream::StreamExt;
use opencv::core::{
    Mat, Mat2b, MatTraitConst, MatTraitConstManual, ToInputArray, Vector, VectorToVec,
};
use opencv::imgcodecs::imencode;
use opencv::types::VectorOfu8;
use opencv::{core, imgcodecs};

use opencv::imgproc;

use actix_web::web::{Bytes, Data};
use chrono::{DateTime, Utc};
use futures_util::{Stream, TryStreamExt};
use std::sync::{mpsc, Arc, Mutex};
use std::task::Poll;
use std::thread;
use tokio::io::{AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::select;
use tokio::sync::broadcast::Sender;
use tokio::sync::RwLock;

pub async fn server(rgb_send: Sender<Bytes>, depth_send: Sender<Bytes>) {
    let video_stream = UdpSocket::bind("0.0.0.0:5000").await.unwrap();

    tracing::info!("Reading frames from camera");



    let mut buf = BytesMut::with_capacity(2048);

    loop {

        let n = video_stream.recv_buf(&mut buf).await.unwrap();
        if (n == 0){
            continue;
        }
        let packet = match Packet::unmarshal(&mut buf){
          Ok(packet) => packet,
            Err(err)=>{
                warn!("Failed to Decode");
                continue;
            }
        };

        let image = packet.payload;

        if (rgb_send.receiver_count() > 0){
            rgb_send.send(image).unwrap();
        }
    }
}
