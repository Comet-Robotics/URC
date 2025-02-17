use std::net::TcpStream;
use std::process::{Command, Stdio, Child};
use std::io::{self, Write};
use std::thread;
use std::time::{Duration, Instant};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use prost::Message as _;
use rover_msgs::rover::{self, Message};
use rand::Rng;
use rover_msgs::rover::*;

const RETRY_DELAY: Duration = Duration::from_secs(1);
// Add starting GPS coordinates (Example: Logan, Utah)
const STARTING_LAT: f64 = 41.745161;
const STARTING_LON: f64 = -111.809472;
const STARTING_ALT: f64 = 1382.0; // meters
const GPS_MOVEMENT_SCALE: f64 = 0.0001; // Approximately 11 meters at this latitude

struct GpsState {
    latitude: f64,
    longitude: f64,
    altitude: f64,
}

impl GpsState {
    fn new() -> Self {
        Self {
            latitude: STARTING_LAT,
            longitude: STARTING_LON,
            altitude: STARTING_ALT,
        }
    }

    fn update_position(&mut self, rng: &mut rand::rngs::ThreadRng) {
        self.latitude += rng.gen_range(-GPS_MOVEMENT_SCALE..GPS_MOVEMENT_SCALE);
        self.longitude += rng.gen_range(-GPS_MOVEMENT_SCALE..GPS_MOVEMENT_SCALE);
        self.altitude += rng.gen_range(-0.5..0.5); // Half meter variation in altitude
    }
}

struct VideoStreamConfig {
    ip: String,
    port: u16,
    width: u32,
    height: u32,
    framerate: u32,
    cpu_usage: i8, // Add cpu_usage
}

impl VideoStreamConfig {
    fn new(ip: &str, port: u16) -> Self {
        Self {
            ip: ip.to_string(),
            port,
            width: 640,
            height: 480,
            framerate: 30,
            cpu_usage: 5, // Default CPU usage value
        }
    }

    fn rtp_url(&self) -> String {
        format!("rtp://{}:{}?pkt_size=1200", self.ip, self.port)
    }
}

struct FFmpegProcess {
    process: Child,
}

impl Drop for FFmpegProcess {
    fn drop(&mut self) {
        println!("Terminating ffmpeg process...");
        // Try to kill the process gracefully
        if let Err(e) = self.process.kill() {
            eprintln!("Failed to kill ffmpeg process: {}", e);
        }
        // Wait for the process to fully terminate
        if let Err(e) = self.process.wait() {
            eprintln!("Failed to wait for ffmpeg process: {}", e);
        }
    }
}

struct StreamManager {
    processes: Vec<FFmpegProcess>,
}

impl Drop for StreamManager {
    fn drop(&mut self) {
        println!("Cleaning up all ffmpeg processes...");
        // FFmpegProcess's Drop implementation will handle each process
    }
}

impl StreamManager {
    fn new() -> Self {
        Self {
            processes: Vec::new(),
        }
    }

    fn add_stream(&mut self, config: &VideoStreamConfig) -> io::Result<()> {
        let process = Command::new("ffmpeg")
            .arg("-re")
            .arg("-f")
            .arg("lavfi")
            .arg("-i")
            .arg(format!(
                "testsrc=size={}x{}:rate={}",
                config.width, config.height, config.framerate
            ))
            .arg("-preset")
            .arg("ultrafast")
            .arg("-vcodec")
            .arg("libx264")
            .arg("-deadline")
            .arg("realtime")
            .arg("-g")
            .arg("10")
            .arg("-error-resilient")
            .arg("1")
            .arg("-auto-alt-ref")
            .arg("1")
            .arg("-f")
            .arg("rtp")
            .arg(config.rtp_url())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .map_err(|e| io::Error::new(
                e.kind(),
                format!("Failed to start ffmpeg: {}", e)
            ))?;

        self.processes.push(FFmpegProcess { process });
        Ok(())
    }
}



fn main() -> Result<(), Box<dyn std::error::Error>> {
    // // Create stream manager to handle ffmpeg processes
    let mut stream_manager = StreamManager::new();

    // Start video streams
    let mut stream1 = VideoStreamConfig::new("0.0.0.0", 5000);
    let mut stream2 = VideoStreamConfig::new("0.0.0.0", 5001);

    stream1.cpu_usage = 6; // Set different cpu_usage values for each stream
    stream2.cpu_usage = 7;

    stream_manager.add_stream(&stream1)?;
    stream_manager.add_stream(&stream2)?;

    // Set up ctrl-c handler for graceful shutdown
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("Received Ctrl+C! Initiating shutdown...");
        r.store(false, Ordering::SeqCst);
    })?;

    // Connect to TCP server with retry logic
    let mut stream = loop {
        if !running.load(Ordering::SeqCst){
            return Ok(());
        }
        match TcpStream::connect("0.0.0.0:8000") {
            Ok(stream) => break stream,
            Err(e) => {
                eprintln!("Failed to connect: {}. Retrying in {:?}...", e, RETRY_DELAY);
                thread::sleep(RETRY_DELAY);
            }
        }
    };
    stream.set_nonblocking(true)?;
    let mut timer = Instant::now();
    let mut gps = GpsState::new();

    // Main message processing loop
    while running.load(Ordering::SeqCst) {
        if (timer.elapsed().as_secs()) >= 2 {
            timer = Instant::now();
            let mut rng = rand::thread_rng();

            let data = ImuData {
                header: None,
                orientation: Some(Quaternion {
                    x: rng.gen_range(-2.0..1.0),
                    y: rng.gen_range(-2.0..1.0),
                    z: rng.gen_range(-2.0..1.0),
                    w: rng.gen_range(-2.0..1.0) 
                }),
                orientation_covariance: vec![],

                angular_velocity: Some(
                    Vector3 {    
                        x: rng.gen_range(-2.0..1.0),
                        y: rng.gen_range(-2.0..1.0),
                        z: rng.gen_range(-2.0..1.0),
                    }
                ),
                angular_velocity_covariance: vec![],
                linear_acceleration:  Some(
                    Vector3 {    
                        x: rng.gen_range(-2.0..1.0),
                        y: rng.gen_range(-2.0..1.0),
                        z: rng.gen_range(-2.0..1.0),
                    }
                ),
                linear_acceleration_covariance: vec![],
            };
            let msg = Message{
                data_type:Some(message::DataType::Imu(data))
            };
            println!("Sending message: {:?}", msg);
            let mut buf = Vec::new();
            let sz = msg.encoded_len() ;
            buf.reserve(sz);

            msg.encode(&mut buf)?;
            stream.write_all(&buf)?;


            let data = GpsData {
                latitude: gps.latitude,
                longitude: gps.longitude,
                altitude: gps.altitude,
                ground_speed: 0.0,
                satellites: 0,
                mode_indicator: 0,
                separation: 0.0,
                true_course: 0.0,
                true_course_magnetic: 0.0,
                dilution: 0.0,
                utc_time: 0,
            };
            let msg = Message{
                data_type:Some(message::DataType::Gps(data))
            };
            println!("Sending message: {:?}", msg);
            let mut buf = Vec::new();
            let sz = msg.encoded_len() ;
            buf.reserve(sz);

            msg.encode(&mut buf)?;
            stream.write_all(&buf)?;
            gps.update_position(&mut rng);
        }


        // Small sleep to prevent tight CPU loop
        thread::sleep(Duration::from_millis(1));
    }

    println!("Shutting down...");
    // stream_manager will be dropped here, cleaning up ffmpeg processes

    Ok(())
}
