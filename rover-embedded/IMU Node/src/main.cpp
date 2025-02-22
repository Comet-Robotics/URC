#include <Arduino.h>
#include <Adafruit_BNO08x.h>

float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
float orientX = 0.0, orientY = 0.0, orientZ = 0.0, orientW = 1.0, orientAccuracy = 0;

#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
void setReports(){
  //Serial.println("ran");
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
  ///  Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    //Serial.println("Could not enable linear acceleration");
  }
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    //Serial.println("Could not enable rotation vector");
  }
}

void setup(void) {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    //Serial.println("pwetty pwease"); // will pause Zero, Leonardo, etc until Serial console opens

  Serial1.begin(115200);
  //Serial.println("Adafruit BNO08x test!");
  

  // Try to initialize!
  if (!bno08x.begin_I2C(0x4B)) {
    //Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("BNO08x Found!");
  setReports();

  //Serial.println("Reading events");
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10);
  if (bno08x.wasReset()) {
    //Serial.print("sensor was reset ");
    setReports();
  }

      while (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_GYROSCOPE_CALIBRATED:
                // Store gyroscope data
                gyroX = sensorValue.un.gyroscope.x;
                gyroY = sensorValue.un.gyroscope.y;
                gyroZ = sensorValue.un.gyroscope.z;
                break;

            case SH2_LINEAR_ACCELERATION:
                // Store linear acceleration data
                accelX = sensorValue.un.linearAcceleration.x;
                accelY = sensorValue.un.linearAcceleration.y;
                accelZ = sensorValue.un.linearAcceleration.z;
                break;

            case SH2_ROTATION_VECTOR:
                // Store orientation data
                orientX = sensorValue.un.rotationVector.i;
                orientY = sensorValue.un.rotationVector.j;
                orientZ = sensorValue.un.rotationVector.k;
                orientW = sensorValue.un.rotationVector.real;
                orientAccuracy = sensorValue.un.rotationVector.accuracy;
                break;

            default:
                break;
        }
      }
    
    Serial.print(gyroX); Serial.print(",");
    Serial.print(gyroY); Serial.print(",");
    Serial.print(gyroZ);
    Serial.print(":");
    Serial.print(accelX); Serial.print(",");
    Serial.print(accelY); Serial.print(",");
    Serial.print(accelZ);
    Serial.print(":");
    Serial.print(orientX); Serial.print(",");
    Serial.print(orientY); Serial.print(",");
    Serial.print(orientZ); Serial.print(","); 
    Serial.print(orientW); Serial.print(","); 
    Serial.println(orientAccuracy);
    
    delay(100);
  
}

