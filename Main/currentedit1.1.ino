//v4
#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

#define DXL_SERIAL Serial1 //defining dynamixel settings
#define DXL_DIR_PIN -1
#define DXL_PROTOCOL_VERSION 2.0

#define setpoint 60 //target angular velocity

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); //changing name of dynamixel library and initializing serial and pin
Adafruit_BMP280 bmp;

unsigned long previousmillis = 0; //previous iterations begin time to calculate loop duration

int nposition[] =  {0,0,0,0}; //neutral position of servos
float prev_ut = 10;

float pitchrate,yawrate,rollrate; //pitch is about x axis, yaw is about y axis and roll is about z axis
float pitcherror,yawerror,rollerror; //initial zero error of the mpu9250
float accx,accy,accz; 
float pitchangle,yawangle; //initial calculation based only on acceleration data
float filteredpitch = 0,filteredyaw = 0,filteredpitcherror = 4,filteredyawerror = 4; //intial values of filtered angles, standard deviation of angle is kept at 2
float filteroutput[2] = {0,0};

float prev_altitude = 0;
float curr_altitude = 0;
float temperature = 0;
float velocity = 0;

float prev_roll_rate = 0; //for roll_reversal_check
float prev_deflection = 0;
float curr_deflection = 0;

float kp = 0; //pi controller variables
float ki = 0;  
float prev_int = 0;  
float error[2] = {0,0};  
#define samptime 0.1

void anglefilter(float filterstate, float filtererror, float gyrorate, float accelangle) { //filterstate = angle from filter
  filterstate = filterstate + 0.05*gyrorate; //initial guess 
  filtererror = filtererror + 0.05*0.05*4*4;
  float gain = filtererror/(filtererror + 3*3);
  filterstate = filterstate + gain*(accelangle-filterstate);
  filtererror = (1-gain)*filtererror;
  filteroutput[0] = filterstate;
  filteroutput[1] = filtererror;
}

void getgyro(void) { //setting and using mpu9250 thruough wire library instead of mpu9250 library because this way setting filter is easier
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x27);
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
  Wire.write(0x24);
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

bool check_altitude(void) {
  bmp.takeForcedMeasurement();
  curr_altitude = bmp.readAltitude(1013.25);
  if (curr_altitude>920) {
    return 1;
  }
  else {
    return 0;
  }
}

bool check_inclination(void) {
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
  float inclination = atan(sqrt(tan(filteredpitch)*tan(filteredpitch)+tan(filteredyaw)*tan(filteredyaw)));
  if (inclination<=15) {
    return 1;
  }
  else {
    return 0;
  }
}

bool roll_reversal_check(void) { //failsafe to check if change in canards is positively or negatively affecting roll
  getgyro();
  curr_roll_rate = rollrate;
  float delta_roll_rate = curr_roll_rate - prev_roll_rate;
  float delta_deflection = curr_deflection; //curr_deflection is the change in deflection in s domain globalised from calculate_delta function
  float delta_roll_by_delta_def;
  if (delta_deflection != 0) {
    delta_roll_by_delta_def = delta_roll_rate/delta_deflection;
    if (delta_roll_by_delta_def < 0) {
      return 1;
    }
    else {
      return 0;
    }
  }
  else {
    return 1;
  }
}

float calcpid(float err) { //calculate ut
  float ut = 0;
  float integral = (samptime/2)*(err + 2*error[1] + error[2]);
  ut = kp*err + ki*integral + prev_int;
  prev_int = prev_int + integral;
  error[2] = error[1];
  error[1] = err;
  return ut;
}

float calculate_delta(float vel, float s) { //calculate canard deflection
  float ans;
  if (vel<=266.96 && vel>216.12) {
    ans = 1.1711/(0.00069986*s + 0.0012654);
    return ans;
  }
  else if (vel<=216.12 && vel>161.93) {
    ans = 1.1417/(0.001208*s + 0.0015953);
    return ans;
  }
  else if (vel<=161.93 && vel>119.05) {
    ans = 1.1226/(0.0023132*s + 0.0021314);
    return ans;
  }
  else if (vel<=119.05 && vel>73.09) {
    ans = 1.1109/(0.0051208*s + 0.0030882);
    return ans;
  }
  else if (vel<=73.09 && vel>20.41) {
    ans = 1.1033/(0.022036*s + 0.0062811);
    return ans;
  }
  else {
    ans = 0;
    return ans;
  }
  curr_deflection = ans;
}
void pivalues(float vel) { //change kp ki values 
  if (vel<=266.96 && vel>216.12) {
    kp = 0.00208333;
    ki = 0.00014583;
  }
  else if (vel<=216.12 && vel>161.93) {
    kp = 0.00260417;
    ki = 0.00018229;
  }
  else if (vel<=161.93 && vel>119.05) {
    kp = 0.00390625;
    ki = 0.00027344;
  }
  else if (vel<=119.05 && vel>73.09) {
    kp = 0.00488281;
    ki = 0.00034180;
  }
  else if (vel<=73.09 && vel>20.41) {
    kp = 0.01144409;
    ki = 0.00080109;
  }
  else {
    kp = 0;
    ki = 0;
  }
}
void setup() {
  Serial.begin(57600); //begin communication with laptop

  DXL_SERIAL.begin(57600); //begin communication with servos
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int id=1;id<5;id++) {  //initialize servos to check if they work
    if (!dxl.ping(id)) {
      Serial.println("Dynamixel");
      Serial.println(id);
      Serial.println("not found");
      while(1);
    }

  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_EXTENDED_POSITION);
  dxl.torqueOn(id);
  dxl.setGoalPosition(id, nposition[id-1] - 250);
  delay(300);
  dxl.setGoalPosition(id, nposition[id-1] + 250);
  delay(500);
  dxl.setGoalPosition(id, nposition[id-1]);
  }

  Wire.setClock(400000); //changing I2C clock speed from default 100kHz to 400kHz
  Wire.begin(); //starting I2C bus communication
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
  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X8,
                  Adafruit_BMP280::STANDBY_MS_1);
}

void loop() {
  unsigned long currentmillis = millis();
  if (currentmillis-previousmillis>=50) { //checking if 50ms has passed since the beginning of the last iteration
    previousmillis = currentmillis;
    if (check_altitude()) { //checking if the altitude is higher than altitude at burnout
      if (check_inclination()) { //checking if inclination is less than or equal to 15 degrees
      } 
      if (roll_reversal_check()) {
      }
    }
  }
}
