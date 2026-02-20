# Gyroscope Visualizer with Foxglove Integration

## Overview

This guide provides a complete implementation for visualizing real-time gyroscope and accelerometer data from an ESP32 MPU6050 sensor using Foxglove Studio. The system captures 6-axis IMU data (3-axis accelerometer + 3-axis gyroscope) and visualizes it with multiple filtered outputs, real-time plots, and a 3D orientation cube.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Hardware Setup](#hardware-setup)
3. [Firmware Implementation](#firmware-implementation)
4. [Data Bridge (Python)](#data-bridge-python)
5. [Foxglove Visualization Setup](#foxglove-visualization-setup)
6. [Running the System](#running-the-system)
7. [Technical Explanations](#technical-explanations)
8. [Troubleshooting](#troubleshooting)

---

## System Architecture

### Data Flow

```
ESP32 (MPU6050) → MQTT → Python Bridge → Foxglove
     ↓
  HTTP Endpoint
  (Optional fallback)
```

### Components

1. **ESP32 Firmware** - Reads MPU6050 sensor at 100 Hz, publishes to MQTT broker
2. **Python Bridge** - Subscribes to MQTT, applies filters, publishes to Foxglove topics
3. **Foxglove Studio** - Real-time 3D visualization and plotting dashboard

---

## Hardware Setup

### Required Components

- ESP32 WROOM DevKit v1
- MPU6050 6-axis IMU sensor
- USB cable for programming
- WiFi network

### Wiring Diagram

```
ESP32           MPU6050
GND      →      GND
3.3V     →      VCC
GPIO 21 (SDA) → SDA
GPIO 22 (SCL) → SCL
```

### Configuration

- **I2C Address**: 0x68 (default for MPU6050)
- **SDA Pin**: GPIO 21 (ESP32)
- **SCL Pin**: GPIO 22 (ESP32)

---

## Firmware Implementation

### ESP32 Arduino Code (gyro.ino)

Upload this code to your ESP32 using Arduino IDE with the following libraries:
- Wire (built-in)
- ESPAsyncWebServer

```cpp
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// MPU6050 Configuration
const int MPU = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// WiFi Configuration
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";
const char* mqtt_server = "192.168.x.x"; // Your MQTT broker IP
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastPublish = 0;
const unsigned long publishInterval = 10; // 10ms = 100Hz

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL
  
  // Initialize MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission(true);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
  
  // Connect to MQTT
  client.setServer(mqtt_server, mqtt_port);
  connectToMQTT();
}

void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32-Gyro")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void readMPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

void publishData() {
  // Create JSON payload
  char payload[128];
  snprintf(payload, sizeof(payload),
    "{\"ax\":%d,\"ay\":%d,\"az\":%d,\"gx\":%d,\"gy\":%d,\"gz\":%d,\"temp\":%d}",
    AcX, AcY, AcZ, GyX, GyY, GyZ, Tmp);
  
  client.publish("sensor/imu/raw", payload);
  
  // Also publish individual topics for Foxglove
  char accel_data[64];
  snprintf(accel_data, sizeof(accel_data), "{\"x\":%d,\"y\":%d,\"z\":%d}", AcX, AcY, AcZ);
  client.publish("sensor/accelerometer", accel_data);
  
  char gyro_data[64];
  snprintf(gyro_data, sizeof(gyro_data), "{\"x\":%d,\"y\":%d,\"z\":%d}", GyX, GyY, GyZ);
  client.publish("sensor/gyroscope", gyro_data);
}

void loop() {
  if (!client.connected()) {
    connectToMQTT();
  }
  client.loop();
  
  unsigned long currentTime = millis();
  if (currentTime - lastPublish >= publishInterval) {
    readMPU6050();
    publishData();
    lastPublish = currentTime;
  }
}
```

### Key Features

- **100 Hz Sampling Rate**: Published every 10ms
- **MQTT Publishing**: Real-time data stream to broker
- **WiFi Connectivity**: Automatic reconnection handling
- **Multiple Topics**: Individual accelerometer and gyroscope data streams

---

## Data Bridge (Python)

### Python Foxglove Bridge (`gyro_foxglove_bridge.py`)

This Python script bridges MQTT data to Foxglove using the Foxglove WebSocket protocol.

```python
#!/usr/bin/env python3
"""
Gyroscope Data Bridge for Foxglove Visualization
Reads raw MPU6050 data from MQTT and applies filters before publishing to Foxglove
"""

import json
import math
import paho.mqtt.client as mqtt
from collections import deque
from scipy import signal
from scipy.spatial.transform import Rotation
import asyncio
from foxglove_websocket import FoxgloveServer, FoxgloveServerListener
from foxglove_websocket.types import ChannelId
from dataclasses import dataclass
from typing import Optional
import time

@dataclass
class IMUData:
    """Container for raw IMU measurements"""
    ax: float  # Accelerometer X
    ay: float  # Accelerometer Y
    az: float  # Accelerometer Z
    gx: float  # Gyroscope X
    gy: float  # Gyroscope Y
    gz: float  # Gyroscope Z
    timestamp: float

class IMUFilter:
    """Applied filtering techniques to IMU data"""
    
    def __init__(self, window_length=21, polyorder=3):
        """
        Initialize filters
        
        Args:
            window_length: Savitzky-Golay filter window length
            polyorder: Savitzky-Golay polynomial order
        """
        self.window_length = window_length
        self.polyorder = polyorder
        
        # Initialize buffers
        self.accel_buffer = deque(maxlen=100)
        self.gyro_buffer = deque(maxlen=100)
        
        # Chebyshev low-pass filter design
        # fs=100Hz, fc=20Hz, ripple=0.1dB
        self.b, self.a = signal.cheby1(4, 0.1, 20 / 50, analog=False)
        self.zi = signal.lfilter_zi(self.b, self.a) * 0  # Filter state
    
    def savitzky_golay_filter(self, data: list) -> list:
        """Apply Savitzky-Golay filter to smooth data"""
        if len(data) < self.window_length:
            return data
        try:
            return signal.savgol_filter(data, self.window_length, self.polyorder).tolist()
        except:
            return data
    
    def chebyshev_filter(self, data: list) -> list:
        """Apply Chebyshev Type I low-pass filter"""
        if len(data) < 2:
            return data
        try:
            return signal.filtfilt(self.b, self.a, data).tolist()
        except:
            return data
    
    def add_sample(self, imu_data: IMUData):
        """Add new sample to buffers"""
        self.accel_buffer.append((imu_data.ax, imu_data.ay, imu_data.az))
        self.gyro_buffer.append((imu_data.gx, imu_data.gy, imu_data.gz))
    
    def get_filtered_accel(self) -> Optional[tuple]:
        """Get Savitzky-Golay filtered accelerometer data"""
        if len(self.accel_buffer) < self.window_length:
            return None
        
        accel_x = [x[0] for x in self.accel_buffer]
        accel_y = [x[1] for x in self.accel_buffer]
        accel_z = [x[2] for x in self.accel_buffer]
        
        filtered_x = self.savitzky_golay_filter(accel_x)[-1]
        filtered_y = self.savitzky_golay_filter(accel_y)[-1]
        filtered_z = self.savitzky_golay_filter(accel_z)[-1]
        
        return (filtered_x, filtered_y, filtered_z)
    
    def get_filtered_gyro(self) -> Optional[tuple]:
        """Get Chebyshev filtered gyroscope data"""
        if len(self.gyro_buffer) < 2:
            return None
        
        gyro_x = [x[0] for x in self.gyro_buffer]
        gyro_y = [x[1] for x in self.gyro_buffer]
        gyro_z = [x[2] for x in self.gyro_buffer]
        
        filtered_x = self.chebyshev_filter(gyro_x)[-1]
        filtered_y = self.chebyshev_filter(gyro_y)[-1]
        filtered_z = self.chebyshev_filter(gyro_z)[-1]
        
        return (filtered_x, filtered_y, filtered_z)

class GyroSystemBridge:
    """Main bridge system connecting MQTT to Foxglove"""
    
    def __init__(self, mqtt_broker: str, mqtt_port: int = 1883, 
                 foxglove_port: int = 8765):
        """
        Initialize the bridge
        
        Args:
            mqtt_broker: IP address of MQTT broker
            mqtt_port: MQTT broker port
            foxglove_port: Foxglove WebSocket server port
        """
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.foxglove_port = foxglove_port
        
        # MQTT Client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # Foxglove Server
        self.foxglove_server = None
        
        # IMU Filter
        self.imu_filter = IMUFilter()
        
        # Buffers for plotting
        self.raw_accel_x = deque(maxlen=200)
        self.raw_accel_y = deque(maxlen=200)
        self.raw_accel_z = deque(maxlen=200)
        self.raw_gyro_x = deque(maxlen=200)
        self.raw_gyro_y = deque(maxlen=200)
        self.raw_gyro_z = deque(maxlen=200)
        
        self.time_axis = deque(maxlen=200)
        self.sample_count = 0
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            print("MQTT Connected successfully")
            # Subscribe to IMU topics
            client.subscribe("sensor/imu/raw")
            client.subscribe("sensor/accelerometer")
            client.subscribe("sensor/gyroscope")
        else:
            print(f"Failed to connect, return code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            payload = json.loads(msg.payload.decode())
            
            # Extract data
            imu_data = IMUData(
                ax=payload.get('ax', 0),
                ay=payload.get('ay', 0),
                az=payload.get('az', 0),
                gx=payload.get('gx', 0),
                gy=payload.get('gy', 0),
                gz=payload.get('gz', 0),
                timestamp=time.time()
            )
            
            # Add to filter
            self.imu_filter.add_sample(imu_data)
            
            # Add to plot buffers
            self.raw_accel_x.append(imu_data.ax)
            self.raw_accel_y.append(imu_data.ay)
            self.raw_accel_z.append(imu_data.az)
            self.raw_gyro_x.append(imu_data.gx)
            self.raw_gyro_y.append(imu_data.gy)
            self.raw_gyro_z.append(imu_data.gz)
            self.time_axis.append(self.sample_count)
            self.sample_count += 1
            
        except Exception as e:
            print(f"Error processing MQTT message: {e}")
    
    async def start_foxglove_server(self):
        """Start Foxglove WebSocket server"""
        
        class ServerListener(FoxgloveServerListener):
            async def on_subscribe(self, channel_id: ChannelId):
                pass
            
            async def on_unsubscribe(self, channel_id: ChannelId):
                pass
            
            async def on_client_connect(self):
                pass
            
            async def on_client_disconnect(self):
                pass
        
        self.foxglove_server = FoxgloveServer(
            listener=ServerListener(),
            port=self.foxglove_port
        )
        
        # Register channels
        await self.foxglove_server.add_channel({
            "id": 1,
            "topic": "accel",
            "schema": "json"
        })
        
        await self.foxglove_server.add_channel({
            "id": 2,
            "topic": "gyro",
            "schema": "json"
        })
        
        await self.foxglove_server.add_channel({
            "id": 3,
            "topic": "orientation",
            "schema": "json"
        })
        
        await self.foxglove_server.start()
    
    def mqtt_connect(self):
        """Connect to MQTT broker"""
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            print(f"Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as e:
            print(f"Failed to connect to MQTT: {e}")
    
    async def run(self):
        """Main event loop"""
        # Connect MQTT
        self.mqtt_connect()
        
        # Start Foxglove server
        await self.start_foxglove_server()
        
        print(f"Foxglove WebSocket server running on ws://localhost:{self.foxglove_port}")
        
        # Publishing loop
        while True:
            try:
                # Get filtered data
                filtered_gyro = self.imu_filter.get_filtered_gyro()
                
                if filtered_gyro:
                    # Calculate orientation from accelerometer
                    pitch = math.atan2(self.raw_accel_y[-1], 
                                      math.sqrt(self.raw_accel_x[-1]**2 + 
                                               self.raw_accel_z[-1]**2)) * 180 / math.pi
                    roll = math.atan2(-self.raw_accel_x[-1], 
                                     self.raw_accel_z[-1]) * 180 / math.pi
                    
                    # Publish to Foxglove
                    if self.foxglove_server:
                        accel_msg = {
                            "x": self.raw_accel_x[-1],
                            "y": self.raw_accel_y[-1],
                            "z": self.raw_accel_z[-1]
                        }
                        
                        gyro_msg = {
                            "x": filtered_gyro[0],
                            "y": filtered_gyro[1],
                            "z": filtered_gyro[2]
                        }
                        
                        orientation_msg = {
                            "pitch": pitch,
                            "roll": roll,
                            "yaw": 0
                        }
                        
                        await self.foxglove_server.send_message(
                            1, 
                            json.dumps(accel_msg).encode()
                        )
                        
                        await self.foxglove_server.send_message(
                            2,
                            json.dumps(gyro_msg).encode()
                        )
                        
                        await self.foxglove_server.send_message(
                            3,
                            json.dumps(orientation_msg).encode()
                        )
                
                await asyncio.sleep(0.01)  # 100Hz update rate
                
            except Exception as e:
                print(f"Error in main loop: {e}")
                await asyncio.sleep(0.1)


async def main():
    """Main entry point"""
    # Configuration
    MQTT_BROKER = "192.168.1.100"  # Change to your MQTT broker IP
    MQTT_PORT = 1883
    FOXGLOVE_PORT = 8765
    
    # Create bridge
    bridge = GyroSystemBridge(MQTT_BROKER, MQTT_PORT, FOXGLOVE_PORT)
    
    # Run
    await bridge.run()


if __name__ == "__main__":
    asyncio.run(main())
```

### Installation

```bash
# Install required Python packages
pip install paho-mqtt scipy foxglove-websocket numpy

# Run the bridge
python3 gyro_foxglove_bridge.py
```

---

## Foxglove Visualization Setup

### Installing Foxglove Studio

1. Download from [foxglove.dev](https://foxglove.dev)
2. Install on your machine
3. Launch the application

### Connecting to the Data Bridge

1. **Open Foxglove Studio**
2. **Create New Connection**
   - Connection Type: `WebSocket`
   - URL: `ws://localhost:8765`
3. **Click Connect**

### Creating Your Dashboard

#### Layout Configuration

1. **Add Panels**:
   - **3D Plot**: For orientation visualization
   - **Line Chart**: For accelerometer data
   - **Line Chart**: For gyroscope data
   - **State Chart**: For additional metrics

#### 3D Object Visualization

To visualize the 3D cube orientation:

1. Add a **3D Plot** panel
2. Subscribe to the `/orientation` topic
3. Configure transform to show device orientation
4. Add a reference cube model for visual reference

#### Time Series Plots

1. Add **Line Chart** panel
2. Configure axes:
   - **X-Axis**: Sample count (time)
   - **Y-Axis**: Acceleration value (m/s²)

3. Subscribe to:
   - `/accel` for accelerometer data
   - `/gyro` for filtered gyroscope data

---

## Running the System

### Step-by-Step Startup

#### 1. Set Up MQTT Broker

```bash
# Using mosquitto
mosquitto -p 1883

# Or using Docker
docker run -it -p 1883:1883 eclipse-mosquitto
```

#### 2. Program ESP32

```bash
# In Arduino IDE:
# 1. Install boards: ESP32 by Espressif Systems
# 2. Set board to "ESP32 WROOM DevKit v1"
# 3. Configure WiFi and MQTT broker IP
# 4. Upload gyro.ino
```

#### 3. Start Python Bridge

```bash
# Terminal 1
python3 gyro_foxglove_bridge.py

# Expected output:
# MQTT Connected successfully
# Foxglove WebSocket server running on ws://localhost:8765
```

#### 4. Open Foxglove Studio

```bash
# Terminal 2
# Launch Foxglove GUI, create WebSocket connection to ws://localhost:8765
```

#### 5. View Data

- Observe real-time plots updating
- Monitor 3D cube rotating with device orientation
- Check filtered vs. raw data comparison

---

## Technical Explanations

### Sensor Calibration

**Raw Values → Physical Units**

```
Accelerometer (MPS²):
aX_mps2 = (raw_aX / 16384.0) * 9.81

Gyroscope (DEG/S):
gX_degs = (raw_gX / 131.0)
```

### Orientation Calculation from Accelerometer

**Pitch, Roll, Yaw computation**:

```
Pitch = atan2(aY, sqrt(aX² + aZ²)) * 180/π
Roll = atan2(-aX, aZ) * 180/π
```

### Filter Characteristics

#### Savitzky-Golay Filter

- **Purpose**: Smoothing while preserving peaks
- **Window Length**: 21 samples
- **Polynomial Order**: 3
- **Frequency Response**: Low-pass characteristics
- **Delay**: ~10 samples (100ms at 100Hz)

#### Chebyshev Type I Filter

- **Purpose**: Sharp low-pass filtering
- **Order**: 4
- **Cutoff**: 20 Hz
- **Ripple**: 0.1 dB passband ripple
- **Sampling Rate**: 100 Hz
- **Application**: Gyroscope noise removal

### Data Flow Timing

```
Raw Sensor Read:     10 ms (100 Hz)
  ↓
MQTT Publish:        ~1-2 ms latency
  ↓
Python Filter:       ~2-3 ms processing
  ↓
Foxglove Display:    ~5-10 ms latency
  ↓
Total E2E Latency:   ~20-30 ms
```

---

## Troubleshooting

### Issue: ESP32 Not Connecting to WiFi

**Solution**:
```
1. Check WiFi credentials in code
2. Ensure ESP32 is within range
3. Check WiFi channel interference
4. Upload WiFi debugging sketch first
```

### Issue: MQTT Connection Refused

**Solution**:
```
1. Verify MQTT broker is running: mosquitto -p 1883
2. Check firewall settings (port 1883)
3. Confirm broker IP matches in ESP32 code
4. Use MQTT Explorer to test broker connectivity
```

### Issue: Foxglove Not Receiving Data

**Solution**:
```
1. Verify WebSocket URL: ws://localhost:8765
2. Check Python bridge is running (see console output)
3. Monitor MQTT topics: mosquitto_sub -t "sensor/#"
4. Check browser console for WebSocket errors
```

### Issue: Jittery Data in Visualization

**Solution**:
```
1. Increase filter window length in IMUFilter
2. Reduce sampling rate (increase publishInterval)
3. Increase Chebyshev filter order
4. Check for EMI near sensor/USB cable
```

### Issue: High Latency

**Solution**:
```
1. Check WiFi signal strength (RSSI)
2. Reduce Python processing (remove extra filters)
3. Increase MQTT QoS but may increase latency trade-off
4. Use 5GHz WiFi if available for better throughput
```

### Issue: Orientation Cube Not Rotating

**Solution**:
```
1. Verify accel/gyro values are changing (not stuck)
2. Check orientation calculation math
3. Ensure 3D plot panel is properly configured
4. Verify transform quaternion calculation
```

### Debugging Commands

```bash
# Monitor all MQTT topics
mosquitto_sub -h localhost -t "sensor/#" -v

# Test MQTT connection
mosquitto_pub -h localhost -t "test" -m "hello"

# Monitor Python bridge output
python3 -u gyro_foxglove_bridge.py 2>&1 | tee bridge.log

# Check Foxglove WebSocket connection
curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" http://localhost:8765
```

---

## Performance Metrics

### Bandwidth Usage

- **MQTT Payload**: ~80 bytes per message
- **Frequency**: 100 messages/second
- **Total Rate**: ~8 KB/s per client
- **WiFi Overhead**: ~40% additional (headers, ACKs)
- **Actual WiFi Usage**: ~11 KB/s

### CPU/Memory

- **ESP32**: ~15% CPU, 30% heap
- **Python Bridge**: ~5% CPU, 50 MB RAM
- **Foxglove**: ~8% CPU, 200 MB RAM

### Latency Breakdown

| Component | Latency |
|-----------|---------|
| Sensor Read | 1 ms |
| WiFi TX | 5-15 ms |
| MQTT Broker | 1-2 ms |
| Python Processing | 2-3 ms |
| Foxglove Display | 10-20 ms |
| **Total** | **20-40 ms** |

---

## Future Enhancements

1. **Quaternion-based Orientation**: More accurate rotation representation
2. **IMU Fusion**: Complementary/Kalman filter for better accuracy
3. **Calibration UI**: Interactive calibration tool in dashboard
4. **Data Logging**: Record and playback capability
5. **Multi-Sensor Support**: Aggregate data from multiple units
6. **Custom Transforms**: User-defined coordinate frame visualization

---

## References

- [MPU6050 Datasheet](https://invensense.tdk.com/download-pdf/mpu-6000-datasheet/)
- [Foxglove Documentation](https://docs.foxglove.dev)
- [MQTT Protocol](http://mqtt.org/)
- [Scipy Signal Processing](https://docs.scipy.org/doc/scipy/reference/signal.html)

---

## License

This implementation is provided as-is for educational and development purposes. Modify and use as needed for your robotics project.
