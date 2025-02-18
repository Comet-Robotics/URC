import socket
import subprocess
import threading
import time
import signal
import sys
import random
import struct
import os

current_dir = os.path.dirname(os.path.abspath(__file__))

# Get the parent directory
parent_dir = os.path.dirname(current_dir)+"/rover-msgs/python"

# Add the parent directory to sys.path
sys.path.append(parent_dir)

# Now you can import your module

from google.protobuf.message import Message
import msgs_pb2 as rover

# Constants
RETRY_DELAY = 1  # seconds
STARTING_LAT = 41.745161
STARTING_LON = -111.809472
STARTING_ALT = 1382.0  # meters
GPS_MOVEMENT_SCALE = 0.0001  # Approximately 11 meters at this latitude


class GpsState:
    def __init__(self):
        self.latitude = STARTING_LAT
        self.longitude = STARTING_LON
        self.altitude = STARTING_ALT

    def update_position(self):
        self.latitude += random.uniform(-GPS_MOVEMENT_SCALE, GPS_MOVEMENT_SCALE)
        self.longitude += random.uniform(-GPS_MOVEMENT_SCALE, GPS_MOVEMENT_SCALE)
        self.altitude += random.uniform(-0.5, 0.5)  # Half meter variation in altitude


class VideoStreamConfig:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.width = 640
        self.height = 480
        self.framerate = 30
        self.cpu_usage = 5  # Default CPU usage value

    def rtp_url(self):
        return f"rtp://{self.ip}:{self.port}?pkt_size=1200"


class FFmpegProcess:
    def __init__(self, config):
        self.config = config
        self.process = None

    def start(self):
        command = [
            "ffmpeg",
            "-re",
            "-f",
            "lavfi",
            "-i",
            f"testsrc=size={self.config.width}x{self.config.height}:rate={self.config.framerate}",
            "-preset",
            "ultrafast",
            "-vcodec",
            "libx264",
            "-deadline",
            "realtime",
            "-g",
            "10",
            "-error-resilient",
            "1",
            "-auto-alt-ref",
            "1",
            "-f",
            "rtp",
            self.config.rtp_url(),
        ]
        try:
            self.process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"FFmpeg process started for {self.config.rtp_url()}")
        except Exception as e:
            print(f"Failed to start ffmpeg: {e}")

    def stop(self):
        if self.process:
            print(f"Terminating ffmpeg process for {self.config.rtp_url()}...")
            try:
                self.process.terminate()
                self.process.wait()
                print(f"FFmpeg process terminated for {self.config.rtp_url()}")
            except Exception as e:
                print(f"Failed to terminate ffmpeg process: {e}")


class StreamManager:
    def __init__(self):
        self.processes = []

    def add_stream(self, config):
        ffmpeg_process = FFmpegProcess(config)
        ffmpeg_process.start()
        self.processes.append(ffmpeg_process)

    def stop_all(self):
        print("Cleaning up all ffmpeg processes...")
        for process in self.processes:
            process.stop()


def create_imu_data():
    imu_data = rover.IMUData()
    imu_data.orientation.x = random.uniform(-2.0, 1.0)
    imu_data.orientation.y = random.uniform(-2.0, 1.0)
    imu_data.orientation.z = random.uniform(-2.0, 1.0)
    imu_data.orientation.w = random.uniform(-2.0, 1.0)

    imu_data.angular_velocity.x = random.uniform(-2.0, 1.0)
    imu_data.angular_velocity.y = random.uniform(-2.0, 1.0)
    imu_data.angular_velocity.z = random.uniform(-2.0, 1.0)

    imu_data.linear_acceleration.x = random.uniform(-2.0, 1.0)
    imu_data.linear_acceleration.y = random.uniform(-2.0, 1.0)
    imu_data.linear_acceleration.z = random.uniform(-2.0, 1.0)

    return imu_data


def create_gps_data(gps_state):
    gps_data = rover.GPSData()
    gps_data.latitude = gps_state.latitude
    gps_data.longitude = gps_state.longitude
    gps_data.altitude = gps_state.altitude
    gps_data.ground_speed = 0.0
    gps_data.satellites = 0
    gps_data.mode_indicator = 0
    gps_data.separation = 0.0
    gps_data.true_course = 0.0
    gps_data.true_course_magnetic = 0.0
    gps_data.dilution = 0.0
    gps_data.utc_time = 0

    return gps_data


def main():
    # Create stream manager to handle ffmpeg processes
    stream_manager = StreamManager()

    # Start video streams
    stream1 = VideoStreamConfig("localhost", 5000)
    stream2 = VideoStreamConfig("localhost", 5001)

    stream1.cpu_usage = 6  # Set different cpu_usage values for each stream
    stream2.cpu_usage = 7

    stream_manager.add_stream(stream1)
    stream_manager.add_stream(stream2)

    # Set up ctrl-c handler for graceful shutdown
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        print("Received Ctrl+C! Initiating shutdown...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    # Connect to TCP server with retry logic
    while running:
        try:
            stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            stream.connect(("localhost", 8000))
            stream.setblocking(False)
            break  # Connection successful, exit the loop
        except socket.error as e:
            print(f"Failed to connect: {e}. Retrying in {RETRY_DELAY} seconds...")
            time.sleep(RETRY_DELAY)
    else:
        print("Exited connection loop due to shutdown signal.")
        stream_manager.stop_all()
        return  # Exit if the loop was exited due to a shutdown signal

    timer = time.time()
    gps = GpsState()

    # Main message processing loop
    while running:
        if time.time() - timer >= 2:
            timer = time.time()

            # Create and send IMU data
            imu_data = create_imu_data()
            msg = rover.Message()
            msg.imu.CopyFrom(imu_data)
            print(f"Sending message: {msg}")
            payload = msg.SerializeToString()
            size = len(payload)
            stream.sendall(struct.pack(">I", size))  # Big-endian unsigned integer
            stream.sendall(payload)

            # Create and send GPS data
            gps_data = create_gps_data(gps)
            msg = rover.Message()
            msg.gps.CopyFrom(gps_data)
            print(f"Sending message: {msg}")
            payload = msg.SerializeToString()
            size = len(payload)
            stream.sendall(struct.pack(">I", size))  # Big-endian unsigned integer
            stream.sendall(payload)

            gps.update_position()

        # Small sleep to prevent tight CPU loop
        time.sleep(0.001)

    print("Shutting down...")
    stream_manager.stop_all()
    stream.close()


if __name__ == "__main__":
    main()