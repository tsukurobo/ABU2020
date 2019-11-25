#include <Wire.h>
#include <ros.h>
#include <omni_ros/RawEncoder.h>
#include <omni_ros/RawPower.h>

#include "ise_motor_driver.h"

ros::NodeHandle nh;
unsigned long time_prev = micros();
IseMotorDriver* md[3];
long enc_prev[3] = {0, 0, 0};

void onReceivePower(const omni_ros::RawPower& power_msg) {
  md[0]->setSpeed(power_msg.p1);
  md[1]->setSpeed(power_msg.p2);
  md[2]->setSpeed(power_msg.p3);
}

omni_ros::RawEncoder encoder_msg;
ros::Subscriber<omni_ros::RawPower> power_sub("raw_power", &onReceivePower);
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
  int16_t enc[3];
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
  // 100Hz
  unsigned long time_current = micros();
  if(abs(time_current - time_prev) > 10000) {
    nh.spinOnce();
    publish_encoder();
    time_prev = time_current;
  }

  if(!nh.connected()) {
    md[0]->setSpeed(0);
    md[1]->setSpeed(0);
    md[2]->setSpeed(0);
  }
}

