#include <Wire.h>
#include <ros.h>
#include <pr/RawEncoder.h>
#include <pr/RawPower.h>
#include <std_msgs/Float32.h>
#include "ise_motor_driver.h"

#define MOTOR_NUM 4

ros::NodeHandle nh;
unsigned long time_prev = micros();
IseMotorDriver* md[MOTOR_NUM];
long enc_prev[MOTOR_NUM] = {0, 0, 0, 0};
const double kp = 47.0, ti = 0.2, td = 0.6;
//const double kp = 400.0, kd = 30;
const int max_duty = 100;
int motor_pw[MOTOR_NUM] = {0,0,0,0};
double speed_now[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0}, speed_pre[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0};
float max_speed; //単位[m/s]
const double abs_max_speed = 1.0; //単位[m/s] モーターが出せる最大回転速度
const double enc_resolution = 1024.0; //もしかすると512かも⇒1024でした
const double wheel_radius = 0.05, pi = 3.141592;
const double dt = 0.01; //ループの周期[s]
int power_pre[MOTOR_NUM] = {0,0,0,0};
pr::RawEncoder encoder_msg;
double diff_i[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0};
double goal_vel[MOTOR_NUM] = {0.0, 0.0 ,0.0, 0.0};
//std_msgs::Float32 debug_info;

//ros::Publisher debug_pub("debug_info", &debug_info);
void onReceivePower(const pr::RawPower& power_msg) {

  speed_now[0] = (((double)encoder_msg.e1)/enc_resolution)*2.0*pi*wheel_radius/dt;
  speed_now[1] = (((double)encoder_msg.e2)/enc_resolution)*2.0*pi*wheel_radius/dt;
  speed_now[2] = (((double)encoder_msg.e3)/enc_resolution)*2.0*pi*wheel_radius/dt;
  speed_now[3] = (((double)encoder_msg.e4)/enc_resolution)*2.0*pi*wheel_radius/dt;
  
  if(power_pre[0] == 0 && power_pre[1] == 0 && power_pre[2] == 0 && power_pre[3] == 0){//停止状態からの走行開始及び走行中からの停止に要する時間を短くする
    motor_pw[0] = (int)((max_speed/abs_max_speed)*power_msg.p1);
    motor_pw[1] = (int)((max_speed/abs_max_speed)*power_msg.p2);
    motor_pw[2] = (int)((max_speed/abs_max_speed)*power_msg.p3);
    motor_pw[3] = (int)((max_speed/abs_max_speed)*power_msg.p4);
    diff_i[0] = 0.0; diff_i[1] = 0.0; diff_i[2] = 0.0; diff_i[3] = 0.0;
  }else{
    //PID controll
    goal_vel[0] = (double)power_msg.p1*max_speed*0.01;
    goal_vel[1] = (double)power_msg.p2*max_speed*0.01;
    goal_vel[2] = (double)power_msg.p3*max_speed*0.01;
    goal_vel[3] = (double)power_msg.p4*max_speed*0.01;
    
    for(int i=0; i<MOTOR_NUM; i++){
      //diff_i[i] += goal_vel[i] - speed_now[i];
      //if(abs(goal_vel[i] - speed_now[i]) <= 0.1) diff_i[i] = 0.0;
      diff_i[i] = 0.0;
      motor_pw[i] += kp*(goal_vel[i] - speed_now[i] - td*(speed_now[i] - speed_pre[i]) + diff_i[i]/ti);
    }
    
/*
  motor_pw[0] = (int)(kp*(((double)power_msg.p1)*max_speed*0.01 - speed_now[0]) - kd*(speed_now[0] - speed_pre[0]));
  motor_pw[1] = (int)(kp*(((double)power_msg.p2)*max_speed*0.01 - speed_now[1]) - kd*(speed_now[1] - speed_pre[1]));
  motor_pw[2] = (int)(kp*(((double)power_msg.p3)*max_speed*0.01 - speed_now[2]) - kd*(speed_now[2] - speed_pre[2]));
  motor_pw[3] = (int)(kp*(((double)power_msg.p4)*max_speed*0.01 - speed_now[3]) - kd*(speed_now[3] - speed_pre[3]));
*/
    for(int l=0; l<MOTOR_NUM; l++){
      if(motor_pw[l] > max_duty){
        motor_pw[l] = max_duty;
      }else if(motor_pw[l] < -max_duty){
        motor_pw[l] = -max_duty;
      }
    }
  }

  for(int i=0; i<MOTOR_NUM; i++){
    md[i]->setSpeed(motor_pw[i]);//setSpeed()関数の引数はduty比
  }

  /*md[0]->setSpeed(power_msg.p1);//setSpeed()関数の引数はduty比
  md[1]->setSpeed(power_msg.p2);
  md[2]->setSpeed(power_msg.p3);*/
  //debug_info.data = motor_pw[3];
  //debug_pub.publish(&debug_info);
  for(int i=0; i<MOTOR_NUM; i++){
    speed_pre[i] = speed_now[i];
  }

  power_pre[0] = power_msg.p1;
  power_pre[1] = power_msg.p2;
  power_pre[2] = power_msg.p3;
  power_pre[3] = power_msg.p4;
}


ros::Subscriber<pr::RawPower> power_sub("raw_power", &onReceivePower);
ros::Publisher encoder_pub("raw_encoder", &encoder_msg);

void setup(){
  Wire.begin();

  nh.initNode();
  nh.subscribe(power_sub);
  nh.advertise(encoder_pub);
  //nh.advertise(debug_pub);

  while(!nh.connected()) {
    nh.spinOnce();
    delay(500);
  }

  int addrs[MOTOR_NUM];
  while(!nh.getParam("/const/i2c_addr", addrs, MOTOR_NUM)) {
    nh.logerror("Failed to get param");
    delay(500);
  }

  for(int i=0; i<MOTOR_NUM; i++) {
    md[i] = new IseMotorDriver(addrs[i]);
    enc_prev[i] = md[i]->encoder();
  }

  nh.getParam("/const/spec/max_speed", &max_speed);
  
}

void publish_encoder() {
  int16_t enc[MOTOR_NUM]={0,0,0,0};
  for(int i=0; i<MOTOR_NUM; i++) {
    long enc_current = md[i]->encoder();
    enc[i] = enc_current - enc_prev[i];
    enc_prev[i] = enc_current;
  }

  encoder_msg.e1 = enc[0];
  encoder_msg.e2 = enc[1];
  encoder_msg.e3 = enc[2];
  encoder_msg.e4 = enc[3];

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
    md[3]->setSpeed(0);
  }
  //delay(1);
}
