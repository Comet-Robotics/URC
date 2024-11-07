use std::net::TcpStream;
use std::time::Duration;

use std::thread;
use std::process::{Command, Stdio};
use std::io;

use bincode::error::DecodeError;
use rover_msgs::Message;

fn stream_test_video_rtp(ip: &str, port: u16) -> io::Result<()> {
    let rtp_url = format!("rtp://{}:{}?pkt_size=1200", ip, port);

    Command::new("ffmpeg")
        .arg("-re")
        .arg("-f")
        .arg("lavfi")
        .arg("-i")
        .arg("testsrc=size=640x480:rate=30") // test video source with specified size and frame rate
        .arg("-vcodec")
        .arg("libvpx") // use VP8 codec
        .arg("-cpu-used")
        .arg("5") // quality setting for encoding speed
        .arg("-deadline")
        .arg("1") // encoding deadline for low latency
        .arg("-g")
        .arg("10") // GOP size
        .arg("-error-resilient")
        .arg("1") // error resilience
        .arg("-auto-alt-ref")
        .arg("1") // enable alternate reference frames
        .arg("-f")
        .arg("rtp") // output format
        .arg(rtp_url) // RTP URL
        .stdout(Stdio::null()) // suppress output
        .stderr(Stdio::null()) // suppress error output
        .spawn()?; // spawn the process

    Ok(())
}



fn main() {


    stream_test_video_rtp( "0.0.0.0",5000).unwrap();
    stream_test_video_rtp( "0.0.0.0",5001).unwrap();
    let config = bincode::config::standard();
    let mut stream = TcpStream::connect("0.0.0.0:8000").unwrap();
    stream.set_nonblocking(true).unwrap();
    loop{
        match bincode::serde::decode_from_std_read::<Message,_,_>(&mut stream, config){
            Ok(msg) => {
                println!("{:?}",msg);
            }
            Err(DecodeError::Io { inner, additional }) => {
                if inner.kind() != std::io::ErrorKind::WouldBlock{
                    println!("{:?}",inner);
                    break;
                }
            }
            Err(err)=>{
                println!("{:?}",err);
                break;
            }
        }
    }



    



  
}
