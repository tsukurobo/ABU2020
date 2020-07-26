#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
//debug
//#include <std_msgs/Int64.h>

//constant
const uint8_t ADDR_PICK = 0x20; //pick up motor address of AVR
const uint8_t ADDR_PASS = 0x14; //pass motor address of AVR
const int TOUCH_PIN    = 2;  //touch sensor pin
const int SOLENOID_PIN = 6;  //solenoid pin
const int VALVE_PIN_1  = 10; //valve pin
const int VALVE_PIN_2  = 8; //valve pin
const int ENC_PER_ROT = 4048; //[enc_step/360deg]
const int MAIN_DELAY  = 10;   //[millisec]

//global variable
int order_pick   = 0; //order of pick up ball
int order_launch = 0; //order of launch ball and wind rope
int pw_lower = 0; //motor power of lower hand (-255~255)
int pw_raise = 0; //motor power of raise ball (-255~255)
int pw_wind  = 0; //motor power of wind rope (-255~255)
int target_deg_1 = 0; //degree of lower hand for pick up[deg]
int target_deg_2 = 0; //degree of lower hand for pick up[deg]
int target_omg   = 0; 
int gainP        = 0;
int delay_sol  = 0; //delay time of solonoid on [milli sec]
int delay_hand = 0; //delay time of hand when pick up [milli sec]
int delay_wind = 0; //delay time of wait wind between normal and reverse[milli sec]

//function prottype
void callback(const std_msgs::Int16MultiArray& msg);
void picking_up();
void launching();
void hand_open();
void hand_hold();


//init
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16MultiArray>  sub("pp_order",&callback);
std_msgs::Int16MultiArray data;
ros::Publisher pub("pp_fin",&data);

IseMotorDriver mot_pick(ADDR_PICK);
IseMotorDriver mot_pass(ADDR_PASS);

//debug
//std_msgs::Int64 debug;
//ros::Publisher pubD("debug_tpc",&debug);
  
void setup(){
  Wire.begin(); 
  
  //init node
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  //debug
  //nh.advertise(pubD);

  data.data_length = 2;
  data.data = (int16_t*)malloc(sizeof(int16_t)*2);
  data.data[0] = 0;
  data.data[1] = 0;
     
  //init pin
  pinMode(TOUCH_PIN, INPUT_PULLUP);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(VALVE_PIN_1, OUTPUT);
  pinMode(VALVE_PIN_2, OUTPUT);
  
  //init actuator
  digitalWrite(SOLENOID_PIN, LOW);
  digitalWrite(VALVE_PIN_1, LOW);
  digitalWrite(VALVE_PIN_2, LOW);
  mot_pick.setSpeed(0);
  mot_pass.setSpeed(0);
}

void loop(){
  nh.spinOnce();

  //pick up
  if(order_pick==1) picking_up();

  //launch
  if(order_launch==1) launching();

  //off all actuators
  digitalWrite(SOLENOID_PIN, LOW);
  digitalWrite(VALVE_PIN_1, LOW);
  digitalWrite(VALVE_PIN_2, LOW);
  mot_pick.setSpeed(0);
  mot_pass.setSpeed(0);
  
  delay(MAIN_DELAY); 
}

////////////////////////////////////////////////////////////////////
//pick up///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void picking_up(){
  //value of touch sensor (OFF:LOW ON:HIGH) 
  long enc = 0;
  long enc_old = 0;
  float omg = 0;
  float omg_old = 0;
  
  //init this function
  nh.spinOnce();
  if(order_pick<1) goto RESET; //ここだけ1未満（ラグの影響を回避するため）

  //open hand
  hand_open();

  //lower hand
  do{
    nh.spinOnce();
    if(order_launch<0) goto RESET;

    mot_pick.setSpeed(pw_lower);
    enc = mot_pick.encorder();
  
    delay(MAIN_DELAY);
  }while(target_deg_1*(ENC_PER_ROT/360.0) < enc);
    
  mot_pick.setSpeed(0);

  //wait for hold hand
  delay(delay_hand);
  
  //hold hand
  hand_hold();

  //wait for hold hand
  delay(delay_hand);
  
  //raise hand
  do{
    nh.spinOnce();
    if(order_pick<0) goto RESET;

    enc = mot_pick.encorder();
    mot_pick.setSpeed(pw_raise);
    delay(MAIN_DELAY);
  }while(enc < target_deg_2*(ENC_PER_ROT/360.0));

  mot_pick.setSpeed(0);

  //complete pick up
  data.data[0] = 0;
  pub.publish(&data);
  
RESET:

  //wait for time lag;
  delay(500);
}

////////////////////////////////////////////////////////////////////////
//launch////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
void launching(){
  //value of touch sensor (OFF:LOW ON:HIGH) 
  long enc = 0;

  //init this function
  nh.spinOnce();
  if(order_launch<1) goto RESET; //ここだけ1未満（ラグの影響を回避するため）

  //open hand
  hand_open();

  //wait for hand
  delay(delay_hand);
  
  //lower hand
  do{
    nh.spinOnce();
    if(order_launch<0) goto RESET;

    mot_pick.setSpeed(pw_lower);
    enc = mot_pick.encorder();
    delay(MAIN_DELAY);
  }while(target_deg_1*(ENC_PER_ROT/360.0) < enc);
    
  mot_pick.setSpeed(0);
  
  delay(500);////////////////////////////////////////wait

  //launch ball  
  while(digitalRead(TOUCH_PIN) == LOW){
    nh.spinOnce();
    if(order_launch<0) goto RESET;
    digitalWrite(SOLENOID_PIN,HIGH);
    delay(delay_sol);
    digitalWrite(SOLENOID_PIN,LOW);
    delay(delay_sol*9);
  }

  //complete launch
  //data.data[1] = 2;
  //pub.publish(&data);

  //wait for launching time
  delay(1000);
  
  //wind rope
  do{
    nh.spinOnce();
    if(order_launch<0) goto RESET;

    mot_pass.setSpeed(pw_wind);

    enc = mot_pass.encorder();
  
    delay(MAIN_DELAY);
  }while(digitalRead(TOUCH_PIN) == HIGH);
  
  delay(delay_wind);

  mot_pass.setSpeed(0);

  data.data[0] = 121;
  pub.publish(&data);
    
  //raise hand
  do{
    nh.spinOnce();
    if(order_launch<0) goto RESET;

    enc = mot_pick.encorder();
    mot_pick.setSpeed(pw_raise);

    data.data[0] = enc;
    pub.publish(&data);
    
    delay(MAIN_DELAY);
  }while(enc < target_deg_2*(ENC_PER_ROT/360.0));

  mot_pick.setSpeed(0);
  
  data.data[0] = 242;
  pub.publish(&data);
  
  //wind reverse
  do{
    nh.spinOnce();
    if(order_launch<0) goto RESET;

    mot_pass.setSpeed(-pw_wind);
    enc = mot_pass.encorder();
    
    data.data[0] = enc;
    pub.publish(&data);
    
    delay(MAIN_DELAY);
  }while(enc < -1000);/////////////////////////////////

  mot_pick.setSpeed(0);

  //complete wind
  data.data[1] = 0;
  pub.publish(&data);

RESET:

  //wait for time lag;
  delay(500);
}

//open hand
void hand_open(){
  digitalWrite(VALVE_PIN_1, LOW);
  digitalWrite(VALVE_PIN_2, LOW);
  digitalWrite(VALVE_PIN_1, HIGH);
  digitalWrite(VALVE_PIN_2, LOW);
}

//hold hand
void hand_hold(){
  digitalWrite(VALVE_PIN_1, LOW);
  digitalWrite(VALVE_PIN_2, LOW);
  digitalWrite(VALVE_PIN_1, LOW);
  digitalWrite(VALVE_PIN_2, HIGH);
}

//ros callback function
void callback(const std_msgs::Int16MultiArray& msg){
  order_pick   = msg.data[0];
  order_launch = msg.data[1];
  pw_lower     = msg.data[2];
  pw_raise     = msg.data[3];
  pw_wind      = msg.data[4];
  target_deg_1 = msg.data[5];
  target_deg_2 = msg.data[6];
  delay_sol    = msg.data[7];
  delay_hand   = msg.data[8];
  delay_wind   = msg.data[9];
}
