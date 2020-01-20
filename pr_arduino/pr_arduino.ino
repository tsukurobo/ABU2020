#include <Wire.h>
#include <ros.h>
#include <pr/RawEncoder.h>
#include <pr/RawPower.h>

#include "ise_motor_driver.h"

ros::NodeHandle nh;
unsigned long time_prev = micros();
IseMotorDriver* md[3];
long enc_prev[3] = {0, 0, 0};
const double kp = 40.0, kd = 10.0;
const double kp2 = 45.0;
const double kp3 = 50.0;
const int max_duty = 100;
int motor_pw[3] = {0,0,0};
double speed_now[3] = {0.0,0.0,0.0}, speed_pre[3] = {0.0,0.0,0.0};
const double max_speed = 0.5; //単位[m/s]
const double abs_max_speed = 1.0; //単位[m/s] モーターが出せる最大回転速度
const double enc_resolution = 1024.0; //もしかすると512かも⇒1024でした
const double wheel_radius = 0.05, pi = 3.141592;
const double dt = 0.005; //ループの周期[s]
int power_pre[3] = {0,0,0};
pr::RawEncoder encoder_msg;

void onReceivePower(const pr::RawPower& power_msg) {

  speed_now[0] = (((double)encoder_msg.e1)/enc_resolution)*2.0*pi*wheel_radius/dt;
  speed_now[1] = (((double)encoder_msg.e2)/enc_resolution)*2.0*pi*wheel_radius/dt;
  speed_now[2] = (((double)encoder_msg.e3)/enc_resolution)*2.0*pi*wheel_radius/dt;
  
  if(power_pre[0] == 0 && power_pre[1] == 0 && power_pre[2] == 0){//停止状態からの走行開始及び走行中からの停止に要する時間を短くする
    motor_pw[0] = (int)((max_speed/abs_max_speed)*power_msg.p1);
    motor_pw[1] = (int)((max_speed/abs_max_speed)*power_msg.p2);
    motor_pw[2] = (int)((max_speed/abs_max_speed)*power_msg.p3);
  }else{
    //PD controll
    motor_pw[0] += (int)(kp*(((double)power_msg.p1)*max_speed*0.01 - speed_now[0]) - kd*(speed_now[0] - speed_pre[0]));
    motor_pw[1] += (int)(kp2*(((double)power_msg.p2)*max_speed*0.01 - speed_now[1]) - kd*(speed_now[1] - speed_pre[1]));
    motor_pw[2] += (int)(kp3*(((double)power_msg.p3)*max_speed*0.01 - speed_now[2]) - kd*(speed_now[2] - speed_pre[2]));
/*
  motor_pw[0] = (int)(kp*(((double)power_msg.p1)*max_speed*0.01 - speed_now[0]) - kd*(speed_now[0] - speed_pre[0]));
  motor_pw[1] = (int)(kp*(((double)power_msg.p2)*max_speed*0.01 - speed_now[1]) - kd*(speed_now[1] - speed_pre[1]));
  motor_pw[2] = (int)(kp*(((double)power_msg.p3)*max_speed*0.01 - speed_now[2]) - kd*(speed_now[2] - speed_pre[2]));
*/  
    for(int l=0; l<3; l++){
      if(motor_pw[l] > max_duty){
        motor_pw[l] = max_duty;
      }else if(motor_pw[l] < -max_duty){
        motor_pw[l] = -max_duty;
      }
    }
  }
  
  md[0]->setSpeed(motor_pw[0]);//setSpeed()関数の引数はduty比
  md[1]->setSpeed(motor_pw[1]);
  md[2]->setSpeed(motor_pw[2]);

  /*md[0]->setSpeed(power_msg.p1);//setSpeed()関数の引数はduty比
  md[1]->setSpeed(power_msg.p2);
  md[2]->setSpeed(power_msg.p3);*/

  speed_pre[0] = speed_now[0];
  speed_pre[1] = speed_now[1];
  speed_pre[2] = speed_now[2];

  power_pre[0] = power_msg.p1;
  power_pre[1] = power_msg.p2;
  power_pre[2] = power_msg.p3;
}


ros::Subscriber<pr::RawPower> power_sub("raw_power", &onReceivePower);
ros::Publisher encoder_pub("raw_encoder", &encoder_msg);

void setup(){
  Wire.begin();

  nh.initNode();
  nh.subscribe(power_sub);
  nh.advertise(encoder_pub);

  while(!nh.connected()) {
    nh.spinOnce();
    delay(500);
  }

  int addrs[3];
  while(!nh.getParam("/const/i2c_addr", addrs, 3)) {
    nh.logerror("Failed to get param");
    delay(500);
  }

  for(int i=0; i<3; i++) {
    md[i] = new IseMotorDriver(addrs[i]);
  }
}

void publish_encoder() {
  int16_t enc[3]={0,0,0};
  for(int i=0; i<3; i++) {
    long enc_current = md[i]->encoder();
    enc[i] = enc_current - enc_prev[i];
    enc_prev[i] = enc_current;
  }

  encoder_msg.e1 = enc[0];
  encoder_msg.e2 = enc[1];
  encoder_msg.e3 = enc[2];

  encoder_pub.publish(&encoder_msg);


}

void loop(){
  // dt*1000000Hz
  unsigned long time_current = micros();
  if(abs(time_current - time_prev) > dt*1000000) {
    nh.spinOnce();
    publish_encoder();
    time_prev = time_current;
  }
  //nh.spinOnce();
  if(!nh.connected()) {
    md[0]->setSpeed(0);
    md[1]->setSpeed(0);
    md[2]->setSpeed(0);
  }
  //delay(1);
}
