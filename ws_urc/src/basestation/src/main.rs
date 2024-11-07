mod msgs;
use std::{env::args, io::{BufReader, Error, ErrorKind, Write}, net::{TcpListener, TcpStream}, process::{Child, Command, Stdio}, sync::Arc, thread::{self, sleep}, time::Duration};
use bincode::config;
use geometry_msgs::msg::Twist;
use msgs::{Message};
use rclrs::Publisher;
use sensor_msgs::msg::Image;
use std_msgs::msg::String as StringMsg;

use clap::Parser;

// fn launch_video_stream(ip_address:&str, port: u32) -> Child {
//     let fps = 30;
//     let args = format!("-y -f rawvideo -pixel_format bgr24 -video_size 640x480 -framerate {fps}")+" -i - -vcodec libvpx -vf \"drawtext=:text=\'%{localtime}\':x=10:y=10:fontsize=24:fontcolor=white\" -deadline 1 -g 10 -error-resilient 1 "+ format!("-auto-alt-ref 1 -f rtp rtp://{ip_address}:{port} -sdp_file stream.sdp");
//     let process = Command::new("ffmpeg").args(args.split(" ")).stdin(Stdio::piped()).spawn().unwrap();
//     return process;

// }

struct BaseStationNode {
    node: Arc<rclrs::Node>,
    // _rgb_subscription: Arc<rclrs::Subscription<Image>>,
    // _depth_subscription: Arc<rclrs::Subscription<Image>>,

    twist_pub: Arc<Publisher<Twist>>
}

impl BaseStationNode {
    fn new(context: &rclrs::Context,ip:&str) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "basestation_node")?;



        // let rgb_process = launch_video_stream(ip, 5000);


        // let _rgb_subscription = node.create_subscription(
        //     "/camera/camera/color/image_raw",
        //     rclrs::QOS_PROFILE_DEFAULT,
        //     |msg: Image| {  


        //     },
        // )?;
        let twist_pub = node.create_publisher::<Twist>("cmd_vel_manual", rclrs::QOS_PROFILE_DEFAULT)?;


        // let depth_process = launch_video_stream(ip, 5000);
        // let depth_in = depth_process.stdin.take().unwrap();
        // let _depth_subscription = node.create_subscription(
        //     "/camera/camera/depth/image_rect_raw",
        //     rclrs::QOS_PROFILE_DEFAULT,
        //     |msg: Image| {
        //         depth_in.write_all(msg.)
               

        //     },
        // )?;

        

        Ok(Self {
            node,
            // _rgb_subscription,
            // _depth_subscription ,
            twist_pub
        })
    }



    

    fn process(&self,msg: Message) -> Result<(),rclrs::RclrsError>{
        // println!("recieved Message {msg:?}");
        match msg{
            Message::Twist(twist) => {
                self.twist_pub.publish(twist)?;
            }
        }
        Ok(())



    }
}



fn main() -> Result<(), rclrs::RclrsError> {


    let context = rclrs::Context::new(std::env::args())?;
    
    let ip = "100.100.246.41";
    let republisher = Arc::new(BaseStationNode::new(&context,ip)?);
    let clone_republisher = republisher.clone();
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        
        let config = config::standard();
        loop {
            
            let mut stream = match TcpStream::connect(format!("{ip}:8000")){
                Ok(a)=> a,
                Err(err)=>{
                    println!("Failed to connect err {err:?}");
                    sleep(Duration::from_secs(1));
                    println!("Connecting again...");
                    continue
                }
            };
            println!(
                "Established connection"
            );
            
            loop{
                println!("Recieving Messages");
                let message:Message = match bincode::serde::decode_from_std_read(&mut stream,config) {
                    Ok(a) => a,
                    Err(err)=> {
                        println!("Failed to decode {err:?}");
                        match err{
                            bincode::error::DecodeError::Io { inner: _, additional: _ } =>{
                                break;
                            },
                            

                            _ => {},
                        }
                        continue;
                        
                    }
                };

                if let Err(err) = clone_republisher.process(message){
                    println!("Error {err:?}");
                    continue;
                }

            }

        }
    });
    rclrs::spin(Arc::clone(&republisher.node))
}