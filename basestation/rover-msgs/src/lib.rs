use serde::{Deserialize, Serialize};
use ts_rs::TS;
#[derive(TS)]
#[ts(export)]
#[derive(Serialize,Deserialize,Debug,Clone)]
pub enum Message{
    Twist(Twist),
    IMU(IMU),
    GPS(Vector3)
}
#[derive(TS)]
#[derive(Serialize,Deserialize,Debug,Clone)]
pub struct IMU{
    //Header header
    //geometry_msgs/Quaternion orientation
    pub orientation_covariance: Quaternion, // Row major about x, y, z axes

    //geometry_msgs/Vector3 angular_velocity
    pub  angular_velocity_covariance: Vector3, // Row major about x, y, z axes

    //geometry_msgs/Vector3 linear_acceleration
    pub linear_acceleration_covariance:Vector3 // Row major x, y z 
}

#[derive(TS)]
#[derive(Serialize,Deserialize,Debug,Clone)]
pub struct Vector3{
    pub x: f64,
    pub y: f64,
    pub z: f64
}

#[derive(TS)]
#[derive(Serialize,Deserialize,Debug,Clone)]
pub struct Quaternion{
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64
}

#[derive(TS)]
#[derive(Serialize,Deserialize,Debug,Clone)]
pub struct Twist{
    pub linear: Vector3 ,
    pub angular: Vector3
}
#[derive(TS)]
#[derive(Serialize,Deserialize,Debug,Clone)]
pub struct RoverState{
    pub x: f32,
    pub y: f32,
    pub z:f32

}


