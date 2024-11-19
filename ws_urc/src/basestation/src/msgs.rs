use bincode::{Decode, Encode};
use geometry_msgs::msg::Twist;
use serde::{Deserialize, Serialize};




#[derive(Deserialize,Serialize,Debug)]
pub enum Message{
    Twist(Twist)
}



