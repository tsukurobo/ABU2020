#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>

//constant
const uint8_t ADDR_PICK = 0x20; //pick up motor address of AVR
const uint8_t ADDR_PASS = 0x14; //pass motor address of AVR
const int TOUCH_PIN    = 2;  //touch sensor pin
const int PIN_SOLENOID = 6;  //solenoid pin
const int PIN_VALVE_1  = 10; //valve pin
const int PIN_VALVE_2  = 8; //valve pin
const int MAIN_DELAY  = 10;   //[millisec]
const int SOLENOID_MODE_ON  = 1;
const int SOLENOID_MODE_OFF = 0;
const int VALVE_MODE_OPEN   = 1;
const int VALVE_MODE_HOLD   = 2;
const int ORDER_ENC_GET = 1;

//actuator
int m_pw_pick;
int m_pw_pass;
int sol_mode;
int valve_mode;
//encoder order
int order_enc_pick;
int order_enc_pass;
//sensor
int touch_val;
long enc_pick;
long enc_pass;
//old
int m_pw_pick_old;
int m_pw_pass_old;
int sol_mode_old;
int valve_mode_old;
int touch_val_old;
long enc_pick_old;
long enc_pass_old;

//function prottype
void callback(const std_msgs::Int32MultiArray& msg);

//init
ros::NodeHandle nh;
std_msgs::Int32MultiArray data_pub;
ros::Subscriber<std_msgs::Int32MultiArray> sub("pp_order",&callback);
ros::Publisher pub("pp_sensor",&data_pub);

IseMotorDriver mot_pick(ADDR_PICK);
IseMotorDriver mot_pass(ADDR_PASS);

void setup(){
  Wire.begin(); 
  
  //init node
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  //init publish data
  data_pub.data_length = 3;
  data_pub.data = (int32_t*)malloc(sizeof(int32_t)*3);
  data_pub.data[0] = 0;
  data_pub.data[1] = 0;
  data_pub.data[2] = 0;
     
  //init pin
  pinMode(TOUCH_PIN, INPUT_PULLUP);
  pinMode(PIN_SOLENOID, OUTPUT);
  pinMode(PIN_VALVE_1, OUTPUT);
  pinMode(PIN_VALVE_2, OUTPUT);
  
  //init actuator
  digitalWrite(PIN_SOLENOID, LOW);
  digitalWrite(PIN_VALVE_1, LOW);
  digitalWrite(PIN_VALVE_2, LOW);
  mot_pick.setSpeed(0);
  mot_pass.setSpeed(0);
}

void loop(){
  nh.spinOnce();

  //get sensor
  touch_val = digitalRead(TOUCH_PIN);
  if(order_enc_pick == ORDER_ENC_GET) enc_pick  = mot_pick.encorder();
  if(order_enc_pass == ORDER_ENC_GET) enc_pass  = mot_pass.encorder();
  
  //actuator
  if(m_pw_pick  != m_pw_pick_old)  mot_pick.setSpeed(m_pw_pick);
  if(m_pw_pass  != m_pw_pass_old)  mot_pass.setSpeed(m_pw_pass);
  if(sol_mode   != sol_mode_old)   run_solenoid(sol_mode);
  if(valve_mode != valve_mode_old) run_valve(valve_mode);

  //sensor
  if(touch_val != touch_val_old) data_pub.data[0] = touch_val;
  if(enc_pick != enc_pick_old)   data_pub.data[1] = enc_pick;
  if(enc_pass != enc_pass_old)   data_pub.data[2] = enc_pass;

  //publish sensors data
  if((touch_val != touch_val_old) || (enc_pick != enc_pick_old) || (enc_pass != enc_pass_old)){
    pub.publish(&data_pub);
  }
  
  //update old data
  m_pw_pick_old  = m_pw_pick; 
  m_pw_pass_old  = m_pw_pass;
  sol_mode_old   = sol_mode;
  valve_mode_old = valve_mode;
  touch_val_old  = touch_val;
  enc_pick_old   = enc_pick;
  enc_pass_old   = enc_pass_old;  
  
  delay(MAIN_DELAY); 
}

void run_solenoid(int mode){
  if     (mode == SOLENOID_MODE_ON)  digitalWrite(PIN_SOLENOID,HIGH);
  else if(mode == SOLENOID_MODE_OFF) digitalWrite(PIN_SOLENOID,LOW);
  else                               digitalWrite(PIN_SOLENOID,LOW);
}

void run_valve(int mode){
  //LOW: lock on, HIGH: lock off
  if(mode == VALVE_MODE_OPEN){
    digitalWrite(PIN_VALVE_1, LOW);
    digitalWrite(PIN_VALVE_2, LOW);
    digitalWrite(PIN_VALVE_1, HIGH);
    digitalWrite(PIN_VALVE_2, LOW);
  }else if(mode == VALVE_MODE_HOLD){
    digitalWrite(PIN_VALVE_1, LOW);
    digitalWrite(PIN_VALVE_2, LOW);
    digitalWrite(PIN_VALVE_1, LOW);
    digitalWrite(PIN_VALVE_2, HIGH);
  }else{
    digitalWrite(PIN_VALVE_1, LOW);
    digitalWrite(PIN_VALVE_2, LOW);
  }
}

//ros callback function
void callback(const std_msgs::Int32MultiArray& msg){
  m_pw_pick  = msg.data[0];
  m_pw_pass  = msg.data[1];
  sol_mode   = msg.data[2];
  valve_mode = msg.data[3];
  order_enc_pick = msg.data[4];
  order_enc_pass = msg.data[5];
}
