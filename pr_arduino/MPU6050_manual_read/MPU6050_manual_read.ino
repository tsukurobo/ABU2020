#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h> 

#define MPU_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define RESOL_ACC 16384.0 //1G(~9.8m/s^2)あたりの計測値
#define RESOL_GYRO 131.0  //1°/sあたりの計測値

int axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, tmp;
double ax,ay,az,gx,gy,gz, gz_pre = 0, yaw = 0;
int time_now = 0, time_pre = 0, time_pre2 = 0; 
double dt = 0;
double offset_yaw = 0;

ros::NodeHandle nh;
std_msgs::Float32 rot;
ros::Publisher gyro_pub("gyro",&rot);

double calcOffsetYaw(void){
  double average_offs = 0;
  int sum = 0; 
  
  for (int i=0; i<10;i++){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  while(Wire.available()<14);

  int _axRaw = Wire.read()<<8 | Wire.read(); //各計測値は16bitの値
  int _ayRaw = Wire.read()<<8 | Wire.read();
  int _azRaw = Wire.read()<<8 | Wire.read();
  int _tmp = Wire.read()<<8 | Wire.read();
  int _gxRaw = Wire.read()<<8 | Wire.read();
  int _gyRaw = Wire.read()<<8 | Wire.read();
  int _gzRaw = Wire.read()<<8 | Wire.read();

  sum += _gzRaw;
  }

  average_offs = (int)sum/10.0/RESOL_GYRO;
  return average_offs;
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  //Serial.begin(9600);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0x00); //Wakes up MPU6050
  Wire.endTransmission();

  nh.initNode();
  nh.advertise(gyro_pub);
  
  offset_yaw = calcOffsetYaw();
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  while(Wire.available()<14);

  axRaw = Wire.read()<<8 | Wire.read(); //各計測値は16bitの値
  ayRaw = Wire.read()<<8 | Wire.read();
  azRaw = Wire.read()<<8 | Wire.read();
  tmp = Wire.read()<<8 | Wire.read();
  gxRaw = Wire.read()<<8 | Wire.read();
  gyRaw = Wire.read()<<8 | Wire.read();
  gzRaw = Wire.read()<<8 | Wire.read();

  gz = (double)gzRaw/RESOL_GYRO; //ドリフト対策
  if(abs(gz - offset_yaw)<0.3){
    gz = 0.0;
  }else{
    gz -= offset_yaw;
  }

  time_now = micros(); //台形積分により、角速度から回転角度を求める
  dt = (double)(time_now - time_pre)/1000000.0;
  yaw += (gz+gz_pre)*dt/2.0; 
  time_pre = time_now;
  gz_pre = gz;

  if(time_now - time_pre2 > 25000){
    rot.data = yaw;
    gyro_pub.publish(&rot);
    nh.spinOnce();
    time_pre2 = time_now;
  }
  //Serial.println(yaw); //Serial.println("°");
}
