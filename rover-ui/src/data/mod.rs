



#[cfg(not(feature = "real"))]
pub mod mock_data;

#[cfg(feature = "real")]
mod signal;
pub  mod rtp_data;



#[cfg(not(feature = "real"))]
pub use mock_data as data;


#[cfg(feature = "real")]
pub use rtp_data as data;



use actix_web::web::Bytes;
use tokio::sync::broadcast::Sender;

pub struct AppState {
    pub rgb_frame:Sender<Bytes>,
    pub depth_frame: Sender<Bytes>,

}
