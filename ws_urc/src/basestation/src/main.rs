use std::{env::args, io::{BufReader, Error, ErrorKind, Write}, net::{TcpListener, TcpStream}, process::{Child, Command, Stdio}, sync::Arc, thread::{self, sleep}, time::Duration};
use bincode::config;
use geometry_msgs::msg::Twist;
use rclrs::Publisher;


struct BaseStationNode {
    node: Arc<rclrs::Node>,
    twist_pub: Arc<Publisher<Twist>>
}

impl BaseStationNode {
    fn new(context: &rclrs::Context,ip:&str) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "basestation_node")?;

        let twist_pub = node.create_publisher::<Twist>("cmd_vel_manual", rclrs::QOS_PROFILE_DEFAULT)?;
        

        Ok(Self {
            node,
         
            twist_pub
        })
    }



    

    fn process(&self,msg: Message) -> Result<(),rclrs::RclrsError>{
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