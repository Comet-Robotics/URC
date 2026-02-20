#include<Wire.h>
#include "ESPAsyncWebServer.h"


const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

const char* ssid = "Instrumentation_measurement";
const char* password = "gyroscope";
AsyncWebServer server(80);
IPAddress IP;


//AsyncEventSource events("/events");

//  unsigned long lastTime = 0;
//  unsigned long lastTimeTemperature = 0;
//  unsigned long lastTimeAcc = 0;
//  unsigned long gyroDelay = 10;
//  unsigned long temperatureDelay = 1000;
//  unsigned long accelerometerDelay = 200;

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(9600);

  WiFi.softAP(ssid, password);
  IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

//  Wire.beginTransmission(MPU);
//  Wire.write(0x3B);  
//  Wire.endTransmission(false);
//  Wire.requestFrom(MPU,12,true); 
//  GyX=Wire.read()<<8|Wire.read(); 
//
//  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(200, "text/plain", String(GyX));
//  });

//  server.addHandler(&events);
  server.begin();

}
void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
  
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(AcX);
  Serial.print(" | Y = "); Serial.print(AcY);
  Serial.print(" | Z = "); Serial.println(AcZ); 
  
  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
  Serial.println(" ");

  Serial.print("AP IP address: ");
  Serial.println(IP);

    server.on("/gyro", HTTP_GET, [](AsyncWebServerRequest *request){
          request->send(200, "text/plain", String(GyX)+" "+String(GyY)+" "+String(GyZ)+" "+String(AcX)+" "+String(AcY)+" "+String(AcZ));      
  });
  
//  if ((millis() - lastTime) > gyroDelay) {
//      // Send Events to the Web Server with the Sensor Readings
//      events.send(GyX,"gyro_readings",millis());
//      lastTime = millis();
//   }

  delay(333);
}