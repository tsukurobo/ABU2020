#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include "pr_msg/LoadMsg.h"
//debug
//#include <std_msgs/Float64.h>

//constant
const uint8_t ADDR_HAND = 0x12; //AVR address of lift hand motor
const uint8_t ADDR_HORIZONTAL = 0x13; //horizonal move motor address of AVR
const uint8_t ADDR_VERTICAL = 0x14; //vertical move motor address of AVR
const uint8_t ADDR_CATCH = 0x15; //catch motor address of AVR
const uint8_t ADDR_LIFT = 0x16; //lift of catch motor address of AVR
const int ENC_PER_ROT = 4048; //[enc_step/360deg]
const int MAIN_DELAY  = 10;   //[millisec]

//positoins of each action (expressed by the value of encoder)
const long int HORI_PILLAR = 0;
const long int HORI_BALL = 0;
const long int HORI_TEE = 0;
const long int VERT_UPPER = 0;
const long int VERT_LOWER = 0;
const long int VERT_TEE = 0;
const long int VERT_CENTER = 0;

//global variable
int order_task = 0; //order of auto loading task
int tee_wind = 0;
int tee_set = 0;
int deg_right = 0;
int deg_left = 0;
int num_called = 0; //which ball to load 

//function prottype
void callback(const std_msgs::Int16MultiArray& msg);
void do_nothing();

//init
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16MultiArray>  sub("load_order",&callback);
std_msgs::Int16 data_fin;
ros::Publisher pub("load_fin",&data_fin);

IseMotorDriver mot_hand(ADDR_HAND);
IseMotorDriver mot_hori(ADDR_HORIZONTAL);
IseMotorDriver mot_vert(ADDR_VERTICAL);
IseMotorDriver mot_catch(ADDR_CATCH);
IseMotorDriver mot_lift(ADDR_LIFT);

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
    case 1: standing_by(); break;
  }
  delay(MAIN_DELAY); 
}

//order_task 0: do nothing
void do_nothing(){
  mot_hand.setSpeed(0);
  mot_hori.setSpeed(0);
  mot_vert.setSpeed(0);
  mot_catch.setSpeed(0);
  mot_lift.setSpeed(0);
}

//order_task 1: standing by
void standing_by(){
  long enc = 0;
  long targetv;
  long targeth;

  enc = mot_hori.encorder();

  if(0 <= num_called && num_called < 3) num_called++;
  else if(num_called == 3) num_called = 0;
  if(num_called == 1){
    targetv = VERT_TEE;
    targeth = HORI_BALL;
  }
  //セットした場所まで動く
  //ボールつかむ
  targeth = HORI_TEE;
  targetv = VERT_TEE;
  //ティーにセットする
  //ボール放す
  //set next point
  if(num_called == 2){
    targetv = VERT_UPPER;
    targeth = HORI_BALL;
  }else if(num_called == 3){
    targeth = HORI_BALL;
    targetv = VERT_LOWER;
  }
  //次の場所へ動く

  //complete pick up
  data_fin.data = 0;
  pub.publish(&data_fin);
}

//ros callback function
void callback(const std_msgs::Int16MultiArray& msg){
  order_task = msg.data[0];
  tee_wind = msg.data[1];
  tee_set = msg.data[2];
}
