#include <Dynamixel2Arduino.h>

#define dxl_serial Serial1
#define dxl_dir_pin -1
#define dxl_protocol_version 2.0

int load[4] = {0,0,0,0};
float load_diff_exp[2];
int status[4] = {1,1,1,1};
int prevpos = 0;

Dynamixel2Arduino dxl(dxl_serial, dxl_dir_pin);

//using namespace ControlTableItem;

void setup() {
  dxl_serial.begin(57600);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(dxl_protocol_version);
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
  for (int i=0;i<4;i++) {
    //dxl.setGoalPosition(i+1, prevpos+50);
    load[i] = dxl.readControlTableItem(63, i+1);
    load[i] = map(load[i],-1000,1000,-100,100);
    Serial.print("Load on servo ");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(load[i]);
    Serial.print(" ");
  }
  load_diff_exp[0] = abs((load[0]-load[1])/(sqrt(load[0]*load[0]+load[1]*load[1])));
  load_diff_exp[1] = abs((load[2]-load[3])/(sqrt(load[2]*load[2]+load[3]*load[3])));
  Serial.print("load_diff0:");
  Serial.print(load_diff_exp[0]);
  Serial.print("load_diff1:");
  Serial.print(load_diff_exp[1]);
  if (load[0]>60 || load[1]>60) {
    dxl.setGoalPosition(1, 0);
    dxl.setGoalPosition(2, 0);
    status[0] = 0;
    status[1] = 0;
  }
  else if (load_diff_exp[0]>0.85) {
    dxl.setGoalPosition(1, 0);
    dxl.setGoalPosition(2, 0);
    status[0] = 0;
    status[1] = 0;
  }
  if (load[2]>60 || load[3]>60) {
    dxl.setGoalPosition(3, 0);
    dxl.setGoalPosition(4, 0);
    status[2] = 0;
    status[3] = 0;
  }
  else if (load_diff_exp[1]>0.85) {
    dxl.setGoalPosition(3, 0);
    dxl.setGoalPosition(4, 0);
    status[2] = 0;
    status[3] = 0;
  }
  Serial.println();
  prevpos = dxl.getPresentPosition(1);
  delay(50);
}
