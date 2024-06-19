#include <Wire.h>
#include <Dynamixel2Arduino.h>

Dynamixel2Arduino dxl(Serial1, -1);
int prevpos = 0;

unsigned long previousmillis = 0;
float pitchrate,yawrate,rollrate;
float pitcherror,yawerror,rollerror;
float accx,accy,accz;
float pitchangle,yawangle;
float filteredpitch = 0,filteredyaw = 0,filteredpitcherror = 2,filteredyawerror = 2;
float filteroutput[2] = {0,0};
void anglefilter(float filterstate, float filtererror, float gyrorate, float accelangle) { //filterstate = angle from filter
  filterstate = filterstate + 0.05*gyrorate; //initial guess 
  filtererror = filtererror + 0.05*0.05*2*2;
  float gain = filtererror/(filtererror + 2*2);
  filterstate = filterstate + gain*(accelangle-filterstate);
  filtererror = (1-gain)*filtererror;
  filteroutput[0] = filterstate;
  filteroutput[1] = filtererror;
}
void getgyro(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8|Wire.read();
  int16_t GyroY=Wire.read()<<8|Wire.read();
  int16_t GyroZ=Wire.read()<<8|Wire.read();
  pitchrate = (float)GyroX/16.4;
  yawrate = (float)GyroY/16.4;
  rollrate = (float)GyroZ/16.4;
}
void getaccel(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x18);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1D);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read()<<8|Wire.read();
  int16_t AccYLSB = Wire.read()<<8|Wire.read();
  int16_t AccZLSB = Wire.read()<<8|Wire.read();
  accx = (float)AccXLSB/2048;
  accy = (float)AccYLSB/2048;
  accz = (float)AccZLSB/2048;
}
void getangle(void) {
  pitchangle = atan(accy/sqrt(accx*accx+accz*accz))*1/(3.142/180);
  yawangle = -atan(accx/sqrt(accy*accy+accz*accz))*1/(3.142/180);
}
void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (int i=0;i<2000;i++) {
    getgyro();
    pitcherror+=pitchrate;
    yawerror+=yawrate;
    rollerror+=rollrate;
    delay(1);
  }
  pitcherror/=2000;
  yawerror/=2000;
  rollerror/=2000;
  Serial1.begin(57600);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  for (int i=1;i<5;i++) {
    dxl.torqueOff(i);
    dxl.setOperatingMode(i, OP_EXTENDED_POSITION);
    dxl.torqueOn(i);
    dxl.setGoalPosition(i, 0);
    delay(200);
    dxl.setGoalPosition(i, 512);
    delay(200);
    dxl.setGoalPosition(i, -512);
    delay(400);
    dxl.setGoalPosition(i, 0);
    delay(200);
  }
}
void loop() {
  unsigned long currentmillis = millis();
  if (currentmillis-previousmillis>=50) {
    previousmillis = millis();
    getgyro();
    getaccel();
    getangle();
    pitchrate-=pitcherror;
    yawrate-=yawerror;
    rollrate-=rollerror;
    anglefilter(filteredpitch,filteredpitcherror,pitchrate,pitchangle);
    filteredpitch = filteroutput[0];
    filteredpitcherror = filteroutput[1];
    anglefilter(filteredyaw,filteredyawerror,yawrate,yawangle);
    filteredyaw = filteroutput[0];
    filteredyawerror = filteroutput[1];
    float pitch_corrected = filteredpitch - 1.2;
    float yaw_corrected = filteredyaw + 4.3;
    Serial.print("Pitch:");
    Serial.print(pitch_corrected);
    Serial.print(" Yaw:");
    Serial.print(yaw_corrected);
    float inclination = atan(sqrt((tan(pitch_corrected/57.296)*tan(pitch_corrected/57.296))+(tan(yaw_corrected/57.296)*tan(yaw_corrected/57.296))));
    Serial.print(" Inclination:");
    // float inclination_corrected = inclination*57.296 - 4;
    Serial.println(inclination*57.296);
    //Serial.println(inclination_corrected);
    if ((inclination*57.296)<15.0) {
      for (int i=0;i<4;i++) {
      dxl.setGoalPosition(i+1, prevpos+50);
      }
    }
    prevpos = dxl.getPresentPosition(1);
  }
}

