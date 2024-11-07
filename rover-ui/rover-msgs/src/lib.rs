use serde::{Deserialize, Serialize};


#[derive(Serialize,Deserialize,Debug)]
pub enum Message{
    Twist(Twist),
}
#[derive(Serialize,Deserialize,Debug)]
pub struct Vector3{
    pub x: f64,
    pub y: f64,
    pub z: f64
}
#[derive(Serialize,Deserialize,Debug)]
pub struct Twist{
    pub linear: Vector3 ,
    pub angular: Vector3
}

#[derive(Serialize,Deserialize,Debug)]
pub struct RoverState{
    pub x: f32,
    pub y: f32,
    pub z:f32

}



#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
