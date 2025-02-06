use bincode::{Decode, Encode};
use serde::{Deserialize, Serialize};
use ts_rs::TS;

#[derive(TS)]
#[ts(export)]
#[derive(Serialize,Deserialize,Debug,Clone,Encode, Decode)]
#[serde(tag = "type")]
pub enum Message{
    Twist(Twist),
    IMU(IMU),
    GPS(Vector3)
}

#[derive(TS)]
#[derive(Serialize,Deserialize,Debug,Clone,Encode, Decode)]
pub struct IMU{
    pub orientation_covariance: Quaternion,
    pub angular_velocity_covariance: Vector3,
    pub linear_acceleration_covariance:Vector3
}
#[derive(TS)]
#[derive(Serialize,Deserialize,Debug,Clone,Encode, Decode)]
pub struct Vector3{
    pub x: f64,
    pub y: f64,
    pub z: f64
}

#[derive(TS)]
#[derive(Serialize,Deserialize,Debug,Clone,Encode, Decode)]
pub struct Quaternion{
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64
}

#[derive(TS)]
#[derive(Serialize,Deserialize,Debug,Clone,Encode, Decode)]
pub struct Twist{
    pub linear: Vector3,
    pub angular: Vector3
}

#[derive(TS)]
#[derive(Serialize,Deserialize,Debug,Clone,Encode, Decode)]
pub struct RoverState{
    pub x: f32,
    pub y: f32,
    pub z:f32
}
