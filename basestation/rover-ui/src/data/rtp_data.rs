
use tokio::sync::mpsc::Sender;
use std::sync::Arc;
use anyhow::Result;
use tokio::net::UdpSocket;
use tokio::sync::{mpsc, oneshot};
use webrtc::api::interceptor_registry::register_default_interceptors;
use webrtc::api::media_engine::{MediaEngine, MIME_TYPE_VP8};
use webrtc::api::APIBuilder;
use webrtc::ice_transport::ice_connection_state::RTCIceConnectionState;
use webrtc::ice_transport::ice_server::RTCIceServer;
use webrtc::interceptor::registry::Registry;
use webrtc::peer_connection::configuration::RTCConfiguration;
use webrtc::peer_connection::peer_connection_state::RTCPeerConnectionState;
use webrtc::peer_connection::sdp::session_description::RTCSessionDescription;
use webrtc::rtp_transceiver::rtp_codec::RTCRtpCodecCapability;
use webrtc::track::track_local::track_local_static_rtp::TrackLocalStaticRTP;
use webrtc::track::track_local::{TrackLocal, TrackLocalWriter};
use webrtc::Error;
use crate::data::signal;

pub async fn server(mut recv: mpsc::Receiver<(String,oneshot::Sender<String>)>) -> Result<()>{
    
    let mut m = MediaEngine::default();

    m.register_default_codecs()?;

    // Create a InterceptorRegistry. This is the user configurable RTP/RTCP Pipeline.
    // This provides NACKs, RTCP Reports and other features. If you use `webrtc.NewPeerConnection`
    // this is enabled by default. If you are manually managing You MUST create a InterceptorRegistry
    // for each PeerConnection.
    let mut registry = Registry::new();

    // Use the default set of Interceptors
    registry = register_default_interceptors(registry, &mut m)?;

    // Create the API object with the MediaEngine
    let api = APIBuilder::new()
        .with_media_engine(m)
        .with_interceptor_registry(registry)
        .build();

    // Prepare the configuration
    let config = RTCConfiguration {
        ice_servers: vec![RTCIceServer {
            urls: vec![],
            ..Default::default()
        }],
        ..Default::default()
    };

    // Create a new RTCPeerConnection
    let peer_connection = Arc::new(api.new_peer_connection(config).await?);

    // Create Track that we send video back to browser on
    let video_track = Arc::new(TrackLocalStaticRTP::new(
        RTCRtpCodecCapability {
            mime_type: MIME_TYPE_VP8.to_owned(),
            ..Default::default()
        },
        "rgb_video".to_owned(),
        "rgb_video".to_owned(),
    ));

    let depth_track = Arc::new(TrackLocalStaticRTP::new(
        RTCRtpCodecCapability {
            mime_type: MIME_TYPE_VP8.to_owned(),
            ..Default::default()
        },
        "depth_video".to_owned(),
        "depth_video".to_owned(),
    ));

    // Add this newly created track to the PeerConnection
    let rgb_rtp_sender = peer_connection
        .add_track(Arc::clone(&video_track) as Arc<dyn TrackLocal + Send + Sync>)
        .await?;

    let depth_rtp_sender = peer_connection
        .add_track(Arc::clone(&depth_track) as Arc<dyn TrackLocal + Send + Sync>)
        .await?;

    // Read incoming RTCP packets
    // Before these packets are returned they are processed by interceptors. For things
    // like NACK this needs to be called.
    tokio::spawn(async move {
        let mut rtcp_buf = vec![0u8; 1500];
        while let Ok((_, _)) = rgb_rtp_sender.read(&mut rtcp_buf).await {}
        Result::<()>::Ok(())
    });

    tokio::spawn(async move {
        let mut rtcp_buf = vec![0u8; 1500];
        while let Ok((_, _)) = depth_rtp_sender.read(&mut rtcp_buf).await {}
        Result::<()>::Ok(())
    });

    let (done_tx, mut done_rx) = tokio::sync::mpsc::channel::<()>(1);

    let done_tx1 = done_tx.clone();
    // Set the handler for ICE connection state
    // This will notify you when the peer has connected/disconnected
    peer_connection.on_ice_connection_state_change(Box::new(
        move |connection_state: RTCIceConnectionState| {
            tracing::debug!("Connection State has changed {connection_state}");
            if connection_state == RTCIceConnectionState::Failed {
                let _ = done_tx1.try_send(());
            }
            Box::pin(async {})
        },
    ));

    let done_tx2 = done_tx.clone();
    // Set the handler for Peer connection state
    // This will notify you when the peer has connected/disconnected
    peer_connection.on_peer_connection_state_change(Box::new(move |s: RTCPeerConnectionState| {
        tracing::debug!("Peer Connection State has changed: {s}");

        if s == RTCPeerConnectionState::Failed {
            // Wait until PeerConnection has had no network activity for 30 seconds or another failure. It may be reconnected using an ICE Restart.
            // Use webrtc.PeerConnectionStateDisconnected if you are interested in detecting faster timeout.
            // Note that the PeerConnection may come back from PeerConnectionStateDisconnected.
            tracing::error!("Peer Connection has gone to failed exiting: Done forwarding");
            let _ = done_tx2.try_send(());
        }

        Box::pin(async {})
    }));
   

    get_stream(video_track.clone(),done_tx.clone(),"0.0.0.0:5000").await?;
    get_stream(depth_track.clone(),done_tx.clone(),"0.0.0.0:5001").await?;
    loop {
        //Wait to recieve a local session description from client;
        let (line,sender) = recv.recv().await.unwrap();
       

     

        let desc_data = signal::decode(line.as_str())?;
        let recv_only_offer = serde_json::from_str::<RTCSessionDescription>(&desc_data)?;

        // Create a MediaEngine object to configure the supported codec
        let mut m = MediaEngine::default();

        m.register_default_codecs()?;

        // Create a InterceptorRegistry. This is the user configurable RTP/RTCP Pipeline.
        // This provides NACKs, RTCP Reports and other features. If you use `webrtc.NewPeerConnection`
        // this is enabled by default. If you are manually managing You MUST create a InterceptorRegistry
        // for each PeerConnection.
        let mut registry = Registry::new();

        // Use the default set of Interceptors
        registry = register_default_interceptors(registry, &mut m)?;

        // Create the API object with the MediaEngine
        let api = APIBuilder::new()
            .with_media_engine(m)
            .with_interceptor_registry(registry)
            .build();


        // Prepare the configuration
        let config = RTCConfiguration {
            ice_servers: vec![RTCIceServer {
                urls: vec!["stun:stun.l.google.com:19302".to_owned()],
                ..Default::default()
            }],
            ..Default::default()
        };

        // Create a new RTCPeerConnection
        let peer_connection = Arc::new(api.new_peer_connection(config).await?);


          // Add this newly created track to the PeerConnection
        let rgb_rtp_sender = peer_connection
        .add_track(Arc::clone(&video_track) as Arc<dyn TrackLocal + Send + Sync>)
        .await?;

        let depth_rtp_sender = peer_connection
            .add_track(Arc::clone(&depth_track) as Arc<dyn TrackLocal + Send + Sync>)
            .await?;

        // Read incoming RTCP packets
        // Before these packets are returned they are processed by interceptors. For things
        // like NACK this needs to be called.
        tokio::spawn(async move {
            let mut rtcp_buf = vec![0u8; 1500];
            while let Ok((_, _)) = rgb_rtp_sender.read(&mut rtcp_buf).await {}
            Result::<()>::Ok(())
        });

        tokio::spawn(async move {
            let mut rtcp_buf = vec![0u8; 1500];
            while let Ok((_, _)) = depth_rtp_sender.read(&mut rtcp_buf).await {}
            Result::<()>::Ok(())
        });
        // Set the handler for Peer connection state
        // This will notify you when the peer has connected/disconnected
        peer_connection.on_peer_connection_state_change(Box::new(
            move |s: RTCPeerConnectionState| {
                tracing::debug!("Peer Connection State has changed: {s}");
                Box::pin(async {})
            },
        ));

        // Set the remote SessionDescription
        peer_connection
            .set_remote_description(recv_only_offer)
            .await?;

        // Create an answer
        let answer = peer_connection.create_answer(None).await?;

        // Create channel that is blocked until ICE Gathering is complete
        let mut gather_complete = peer_connection.gathering_complete_promise().await;

        // Sets the LocalDescription, and starts our UDP listeners
        peer_connection.set_local_description(answer).await?;

        // Block until ICE Gathering is complete, disabling trickle ICE
        // we do this because we only can exchange one signaling message
        // in a production application you should exchange ICE Candidates via OnICECandidate
        let _ = gather_complete.recv().await;

        if let Some(local_desc) = peer_connection.local_description().await {
            let json_str = serde_json::to_string(&local_desc)?;
            let b64 = signal::encode(&json_str);
            sender.send(b64).unwrap();
        } else {
            tracing::error!("generate local_description failed!");
        }
    }
}

#[cfg(feature = "real")]
async fn get_stream(video_track: Arc<TrackLocalStaticRTP>,done_tx:Sender<()>,ip:&str)-> Result<()>{
        // Open a UDP Listener for RTP Packets on port 5004
    let listener = UdpSocket::bind(ip).await?;

    // Read RTP packets forever and send them to the WebRTC Client
    tokio::spawn(async move {
        let mut inbound_rtp_packet = vec![0u8; 1600]; // UDP MTU
        while let Ok((n, _)) = listener.recv_from(&mut inbound_rtp_packet).await {
            let data = &inbound_rtp_packet[..n];
            tracing::trace!("Received {} bytes", n);

            if let Err(err) = video_track.write(data).await {
                
                if Error::ErrClosedPipe == err {
                    // The peerConnection has been closed.
                    tracing::error!("Track closed");

                } else {
                    tracing::error!("video_track write err: {err}");
                }
                let _ = done_tx.try_send(());
                return;
            }
        }
    });
    Ok(())
}


#[cfg(not(feature = "real"))]
async fn get_stream(video_track: Arc<TrackLocalStaticRTP>,done_tx:Sender<()>,ip:&str)-> Result<()>{
        // Open a UDP Listener for RTP Packets on port 5004
    let listener = UdpSocket::bind(ip).await?;

    // Read RTP packets forever and send them to the WebRTC Client
    tokio::spawn(async move {
        let mut inbound_rtp_packet = vec![0u8; 1600]; // UDP MTU
        while let Ok((n, _)) = listener.recv_from(&mut inbound_rtp_packet).await {
            let data = &inbound_rtp_packet[..n];
            if let Err(err) = video_track.write(data).await {
                
                if Error::ErrClosedPipe == err {
                    // The peerConnection has been closed.
                    println!("Track closed");

                } else {
                    println!("video_track write err: {err}");
                }
                let _ = done_tx.try_send(());
                return;
            }
        }
    });
    Ok(())
}