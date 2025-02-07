use std::net::TcpStream;
use std::process::{Command, Stdio, Child};
use std::io;
use std::thread;
use std::time::{Duration, Instant};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use bincode::error::DecodeError;
use rover_msgs::Message;
use rand::Rng;

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
            .arg("libvpx")
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

fn handle_message_stream(stream: &mut TcpStream) -> Result<bool, String> {
    let config = bincode::config::standard();
    match bincode::decode_from_std_read::<Message, _,_ >(stream, config) {
        Ok(msg) => {
            println!("Received message: {:?}", msg);
            Ok(true)
        }
        Err(DecodeError::Io { inner, .. }) => {
            match inner.kind() {
                io::ErrorKind::WouldBlock => Ok(true),
                _ => Err(format!("IO Error: {}", inner))
            }
        }
        Err(err) => {
            println!("Decode error: {:?}", err);
            Ok(true)
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create stream manager to handle ffmpeg processes
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

            let msg = Message::IMU(rover_msgs::IMU {
                orientation_covariance: rover_msgs::Quaternion {
                    x: rng.gen_range(-2.0..1.0),
                    y: rng.gen_range(-2.0..1.0),
                    z: rng.gen_range(-2.0..1.0),
                    w: rng.gen_range(-2.0..1.0)
                },
                angular_velocity_covariance: rover_msgs::Vector3 {
                    x: rng.gen_range(-1.0..1.0),
                    y: rng.gen_range(-1.0..1.0),
                    z: rng.gen_range(-1.0..1.0)
                },
                linear_acceleration_covariance: rover_msgs::Vector3 {
                    x: rng.gen_range(-1.0..1.0),
                    y: rng.gen_range(-1.0..1.0),
                    z: rng.gen_range(-1.0..1.0)
                },
            });
            println!("Sending message: {:?}", msg);
            bincode::encode_into_std_write(msg, &mut stream, bincode::config::standard())?;

            gps.update_position(&mut rng);
            let msg = Message::GPS(rover_msgs::Vector3 {
                    x: gps.latitude,
                    y: gps.longitude,
                    z:gps.altitude
                });
            println!("Sending message: {:?}", msg);
            bincode::encode_into_std_write(msg, &mut stream, bincode::config::standard())?;
        }

        match handle_message_stream(&mut stream) {
            Ok(continue_running) => {
                if !continue_running {
                    break;
                }
            }
            Err(e) => {
                eprintln!("Error processing message: {}", e);
                break;
            }
        }

        // Small sleep to prevent tight CPU loop
        thread::sleep(Duration::from_millis(1));
    }

    println!("Shutting down...");
    // stream_manager will be dropped here, cleaning up ffmpeg processes

    Ok(())
}