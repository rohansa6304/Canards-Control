//v3
#include <Adafruit_BMP280.h> //including all libraries 
#include <Dynamixel2Arduino.h>
#include <MPU9250_asukiaaa.h>
#include <Wire.h>

#define DXL_SERIAL Serial1 //defining dynamixel settings
#define DXL_DIR_PIN -1
#define DXL_PROTOCOL_VERSION 2.0

#define setpoint 60 //target angular velocity

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); //changing names of libraries
MPU9250_asukiaaa mpu;
Adafruit_BMP280 bmp;

int nposition[] =  {0,0,0,0}; //neutral position of servos
float prev_alt = 0; //previous iteration values
float prev_ut = 10;

float kp = 0; //pi controller variables
float ki = 0;  
float prev_int = 0;  
float error[2] = {0,0};  
#define samptime 0.1

int data[9] = {0,0,0,0,0,0,0,0,0}; //filter data
float zeroerror = 0;

float calcpid(float err) { //calculate ut
  float ut = 0;
  float integral = (samptime/2)*(err + 2*error[1] + error[2]);
  ut = kp*err + ki*integral + prev_int;
  prev_int = prev_int + integral;
  error[2] = error[1];
  error[1] = err;
  return ut;
}
float filter(float gyro) { //filter angular velocity
  float ans;
  ans = (data[1] + data[2] + data[3] + data[4] + data[5] + gyro)/6;
  data[1] = data[2];
  data[2] = data[3];
  data[3] = data[4];
  data[4] = data[5];
  data[5] = gyro;
  return ans;
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

  Wire.begin(); //I2C bus
  mpu.setWire(&Wire); //initialize imu
  mpu.beginGyro(GYRO_FULL_SCALE_2000_DPS);
  bmp.begin(0x76); //initialize altimeter
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED, //altimeter settings
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X8,
                  Adafruit_BMP280::STANDBY_MS_1);
}

void loop() {
  mpu.gyroUpdate(); //get imu data
  bmp.takeForcedMeasurement(); //get altimeter data

  float altitude = bmp.readAltitude(1002); //check hPa at sea level on that day
  float velocity = (altitude - prev_alt)/0.05;
  float gZ = mpu.gyroZ();
  float avgvalue = filter(gZ);
  int angvel = map(avgvalue, -32768, 32767, -2000, 2000);

  Serial.print("gyro value: "); //just printing stuff
  Serial.print(gZ);
  Serial.print(" avg value: ");
  Serial.print(avgvalue);
  Serial.print(" deg/s: ");
  Serial.print(angvel);
  Serial.print(" Altitude: ");
  Serial.print(altitude);
  Serial.print(" Velocity = ");
  Serial.println(velocity);
  pivalues(velocity);

  float tform = calcpid(setpoint - angvel); //calculating ut
  if (tform>10) { //limiting ut value
    tform = 10;
  }
  if (((tform-prev_ut)/0.05)<-8) { //limiting rate of ut
    tform = prev_ut-0.4;
  }
  float delta = calculate_delta(velocity,tform); //calculate deflection
  int canard = map(delta,-15,15,-171,171); //map degrees to servo position

  for (int id=1;id<5;id++) { //move the canards
    dxl.setGoalPosition(id,nposition[id-1] - canard);
  }

  prev_alt = altitude; //change iteration values
  delay(50); //need to change
}
