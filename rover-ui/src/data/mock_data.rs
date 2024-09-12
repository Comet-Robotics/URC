
use actix_web::web::Bytes;

use tokio::sync::broadcast::Sender;

pub async fn server(rgb_send: Sender<Bytes>, depth_send:  Sender<Bytes>) {


    loop {



    }

}

