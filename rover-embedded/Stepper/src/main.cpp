#include <AccelStepper.h>
#include <MultiStepper.h>
#define zDir 7
#define xDir 5
#define yDir 6
int enable = 8;

#define zStepPin 4
#define xStepPin 2
#define yStepPin 3

AccelStepper stepperz(1, zStepPin, zDir);
AccelStepper stepperx(1, xStepPin, xDir);
AccelStepper steppery(1, yStepPin, yDir);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(zDir, OUTPUT);
  pinMode(xDir, OUTPUT);
  pinMode(yDir, OUTPUT);
  pinMode(zStepPin, OUTPUT);
  pinMode(xStepPin, OUTPUT);
  pinMode(yStepPin,OUTPUT);
  pinMode(enable, OUTPUT);

  stepperz.setMaxSpeed(1000);
  stepperz.setAcceleration(100);
  stepperx.setMaxSpeed(1000);
  stepperx.setAcceleration(100);
  steppery.setMaxSpeed(1000);
  steppery.setAcceleration(100);
  stepperz.setSpeed(500);
  stepperx.setSpeed(500);
  steppery.setSpeed(500);
  digitalWrite(enable, LOW);
}
unsigned long timestamp;
void loop() {
  // put your main code here, to run repeatedly:
  stepperx.setSpeed(200);
  steppery.setSpeed(200);
  stepperz.setSpeed(500);
  // while (Serial.available()){
  //   char cmd[5] = {};
  //   int read = Serial.readBytesUntil(':',cmd,5);

  //   if (read == 0){
  //     continue;
  //   }
  //   int Dir = cmd[1] =='1' ? 1 : 0;
  //   char spd[3] = {cmd[2],cmd[3],cmd[4]};
  //   int speed = atoi(spd) * ((Dir -1)*2 +1);
  //   Serial.println(speed);

  //   if(cmd[0] == 'R'){
  //     stepperx.setSpeed(speed);
  //     steppery.setSpeed(speed);
  //   } else{
  //     stepperz.setSpeed(speed);
  //   }


  // }
  // if (millis()-timestamp > 3000){
  //   speed =-speed;
  //   stepperz.setSpeed(speed);
  //   stepperx.setSpeed(-speed);
  //   timestamp = millis();

  // }

  stepperz.runSpeed();
  stepperx.runSpeed();
  steppery.runSpeed();






}