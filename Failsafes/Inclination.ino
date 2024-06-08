#include <Wire.h>
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
    Serial.print("Pitch:");
    Serial.print(filteredpitch);
    Serial.print(" Yaw:");
    Serial.print(filteredyaw);
    float inclination = atan(sqrt((tan(filteredpitch/57.296)*tan(filteredpitch/57.296))+(tan(filteredyaw/57.296)*tan(filteredyaw/57.296))));
    Serial.print(" Inclination:");
    Serial.println(inclination*57.296);
  }
}
