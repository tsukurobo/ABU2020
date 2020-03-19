#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
//debug
//#include <std_msgs/Float64.h>

//constant
const uint8_t ADDR_HORIZONTAL = 0x13; //horizonal move motor address of AVR
const uint8_t ADDR_VERTICAL = 0x14; //vertical move motor address of AVR
const uint8_t ADDR_CATCH = 0x15; //catch motor address of AVR
const int ENC_PER_ROT = 4048; //[enc_step/360deg]
const int MAIN_DELAY  = 10;   //[millisec]

//global variable
int order_task = 0; //order of auto loading task
int deg_right = 0;
int deg_left = 0;

//function prottype
void callback(const std_msgs::Int16MultiArray& msg);
void do_nothing();

//init
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16MultiArray>  sub("load_order",&callback);
std_msgs::Int16 data_fin;
ros::Publisher pub("load_fin",&data_fin);

IseMotorDriver mot_hori(ADDR_HORIZONTAL);
IseMotorDriver mot_vert(ADDR_VERTICAL);
IseMotorDriver mot_catch(ADDR_CATCH);

//debug
//std_msgs::Float64 debug;
//ros::Publisher pubD("debug_tpc",&debug);
  
void setup(){
  Wire.begin(); 
  
  //init node
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  //debug
  //nh.advertise(pubD);

  data_fin.data = 0;
     
  //init actuator
  mot_hori.setSpeed(0);
  mot_vert.setSpeed(0);
  mot_catch.setSpeed(0);
}


void loop(){
  nh.spinOnce();

  switch(order_task){
    case 0: do_nothing(); break;
  }
  
  
  delay(MAIN_DELAY); 
}

//order_task 0: do nothing
void do_nothing(){
  mot_hori.setSpeed(0);
  mot_vert.setSpeed(0);
  mot_catch.setSpeed(0);
}

//order_task 1: standing by
void standing_by(){
  long enc = 0;

  enc = mot_hori.encorder();

  //complete pick up
  data_fin.data = 0;
  pub.publish(&data_fin);
}

//ros callback function
void callback(const std_msgs::Int16MultiArray& msg){
  order_task = msg.data[0];
}
