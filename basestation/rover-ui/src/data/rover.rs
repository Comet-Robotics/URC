
use std::{
    io::{Read, Write}, 
    thread, 
    time::Duration
};
use rover_msgs::rover::Message;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt}, net::{TcpListener, TcpStream}, sync::{broadcast,mpsc::{self, error::TryRecvError}}
};
use prost::{decode_length_delimiter, length_delimiter_len, Message as _};
use prost::DecodeError;
use tracing::debug;

// ... existing code ...
const TCP_PORT:u16 = 8000;
const BUFFER_SIZE: usize = 1024;


pub async fn launch_rover_link(
    mut msg_rx: mpsc::Receiver<Message>,
    msg_tx: broadcast::Sender<Message>,
) -> Result<(), Box<dyn std::error::Error>> {
    let listener = TcpListener::bind(("0.0.0.0", TCP_PORT)).await?;
    tracing::info!("Rover link launched on port {}", TCP_PORT);

    'connection_loop: loop {
        tracing::info!("Waiting for rover connection...");
        let (mut socket, addr) = listener.accept().await?;
        tracing::info!("New connection: {}", addr);
        
        let (read_half, mut write_half) = socket.into_split();
        
        // Spawn a task to handle incoming messages
        let  msg_tx = msg_tx.clone();
        let read_task = tokio::spawn(async move {
            let mut buf = vec![0u8; BUFFER_SIZE];
            let mut read_half = tokio::io::BufReader::new(read_half);
            
            loop {
                if let Err(err) = read_half.read_exact(&mut buf[..1]).await{
                    tracing::error!("Failed to read message length: {}", err);
                    break;
                }

                let msg = { match decode_length_delimiter(&buf[..1]) {
                        Ok(sz) => {
                            if sz > buf.len() {
                                buf.resize(sz, 0);
                            }
                            read_half.read_exact(&mut buf[..sz]).await.unwrap();
                            Message::decode(&buf[..sz]).unwrap()
                        }
                        Err(_) => {
                            // protobuf 消息的长度信息最少占有 1 byte, 最多占有 10 bytes
                            read_half.read_exact(&mut buf[1..10]).await.unwrap();
                            let sz = decode_length_delimiter(&buf[..10]).unwrap();
                            let delimiter_len = length_delimiter_len(sz);
                            let idx = delimiter_len;
                            let left = sz - (10 - idx);

                            if 10 + left > buf.len() {
                                buf.resize(10 + left, 0);
                            }

                            read_half.read_exact(&mut buf[10..left]).await.unwrap();
                            Message::decode(&buf[idx..idx + sz]).unwrap()
                        }
                    }
                };
        
                // debug!("Recivied Message {msg:?}");
                msg_tx.send(msg).unwrap();
            

            }
            tracing::debug!("Read task finished");
        });
        
        // Handle outgoing messages
        loop {
            if read_task.is_finished(){
                tracing::debug!("Read task finished restarting");
                break;
            }
        
            let msg = msg_rx.try_recv();
            match msg {
                Ok(msg) => {
                    let encoded = msg.encode_to_vec();
                    let length = encoded.len() as u32;
                    tracing::debug!("Sending message");
                    // Send message length first
                    if let Err(e) = write_half.write_all(&length.to_be_bytes()).await {
                        tracing::error!("Failed to write message length: {}", e);
                        continue 'connection_loop;
                    }

                    // Send message content
                    if let Err(e) = write_half.write_all(&encoded).await {
                        tracing::error!("Failed to write message: {}", e);
                        continue 'connection_loop;
                    }

                    if let Err(e) = write_half.flush().await {
                        tracing::error!("Failed to flush: {}", e);
                        continue 'connection_loop;
                    }
                }
                Err(TryRecvError::Empty) => {
                    // No message to send
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
                Err(err) => {
                    tracing::error!("Error receiving message from channe");
                    continue 'connection_loop;
                }
            }
            
            
        }
    }
}