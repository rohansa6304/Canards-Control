#include <MPU9250_asukiaaa.h>
#include <Wire.h>
MPU9250_asukiaaa mpu;
float input = 0;
float output[2] = {0,0};
float filter(float gyro) {
  float ans = (gyro + input + 78*output[0] + 78*output[1])/1601;
  input = gyro;
  output[1] = output[0];
  output[0] = ans;
  return ans;
}
void setup() {
  Serial.begin(57600);
  Wire.begin();
  mpu.setWire(&Wire);
  mpu.beginGyro(GYRO_FULL_SCALE_2000_DPS);
}

void loop() {
  mpu.gyroUpdate();
  float gZ = mpu.gyroZ();
  float filtered = filter(gZ);
  Serial.print("Raw: ");
  Serial.print(gZ);
  Serial.print(" Filtered: ");
  Serial.println(filtered);
  delay(50);
}
