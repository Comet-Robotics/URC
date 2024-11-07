use std::{io::Write, net::TcpListener, sync::mpsc, thread, time::Duration};

use bincode::{error::DecodeError};
use rover_msgs::Message;
use serde::{Deserialize, Serialize};
pub  fn launch_rover_link(mut msg_rx: mpsc::Receiver<Message>) -> Result<(), Box<dyn std::error::Error>>{


    let listener = TcpListener::bind("0.0.0.0:8000")?;

    println!("Rover link launched");



    let config = bincode::config::standard();

    loop{
        let (mut socket,_addr) = match listener.accept(){
            Ok((socket, addr)) => {
                println!("New connection: {}", addr);
                (socket, addr)
            }
            Err(e) => {
                println!("Failed to establish a connection: {}", e);
                continue;
            }
        };
        socket.set_nonblocking(true).unwrap();


        loop{
            match msg_rx.try_recv(){
                Ok(msg) => {
                    println!("Sending message: {:?}", msg);
                    bincode::serde::encode_into_std_write(msg, &mut socket, config).unwrap();
                    socket.flush().unwrap();

                },                
                Err(mpsc::TryRecvError::Empty) => {},
                Err(mpsc::TryRecvError::Disconnected) => return Ok(()),
            };


            match bincode::serde::decode_from_std_read::<Message,_,_>(&mut socket, config){
                Ok(msg) => {
                    println!("Received message: {:?}", msg);


                },
                Err(e) => {
                    if let DecodeError::Io { inner, additional } = &e{
                        if inner.kind() == std::io::ErrorKind::WouldBlock{
                            thread::sleep(Duration::from_millis(10)); // Sleep to avoid CPU overuse
                            continue;
                        }
                    }
                    println!("Failed to receive message: {:?}", e);
                    break;
                }
            }


         
        }

    }



}