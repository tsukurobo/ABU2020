#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

//constant
const uint8_t ADDR = 0x21;  //motor address of AVR
const int PIN_TOUCH    = 6; //touch sensor pin
const int PIN_SOLENOID = 7; //solenoid pin
const int MAIN_DELAY  = 10; //[millisec]
const int SOLENOID_MODE_ON  = 1;
const int SOLENOID_MODE_OFF = 0;
const int ORDER_ENC_GET = 1;

//actuator
int m_pw;
int sol_mode;
//encoder order
int order_enc;
//sensor
int touch_val;
long enc_val;
//old
int m_pw_old;
int sol_mode_old;
int touch_val_old;
long enc_val_old;

//function prottype
void callback(const std_msgs::Int32MultiArray& msg);

//init
ros::NodeHandle nh;
std_msgs::Int32MultiArray data_pub;
ros::Subscriber<std_msgs::Int32MultiArray> sub("kick_order",&callback);
ros::Publisher pub("kick_sensor",&data_pub);

IseMotorDriver mot(ADDR);

void setup(){
  Wire.begin(); 
  
  //init node
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  //init publish data
  data_pub.data_length = 2;
  data_pub.data = (int32_t*)malloc(sizeof(int32_t)*2);
  data_pub.data[0] = 0;
  data_pub.data[1] = 0;
     
  //init pin
  pinMode(PIN_TOUCH, INPUT_PULLUP);
  pinMode(PIN_SOLENOID, OUTPUT);
  
  //init actuator
  digitalWrite(PIN_SOLENOID, LOW);
  mot.setSpeed(0);
}

void loop(){
  nh.spinOnce();

  //get sensor
  touch_val = digitalRead(PIN_TOUCH);
  if(order_enc == ORDER_ENC_GET) enc_val = mot.encorder();
  
  //actuator
  if(m_pw     != m_pw_old)     mot.setSpeed(m_pw);
  if(sol_mode != sol_mode_old) run_solenoid(sol_mode);

  //publish sensors data
  if((touch_val != touch_val_old) || (enc_val != enc_val_old)){
	  data_pub.data[0] = touch_val;
    data_pub.data[1] = enc_val;
	  pub.publish(&data_pub);
  }
  
  //update old data
  m_pw_old      = m_pw;
  sol_mode_old  = sol_mode;
  touch_val_old = touch_val;
  enc_val_old   = enc_val;  
  
  delay(MAIN_DELAY); 
}

//solenoid
void run_solenoid(int mode){
  if     (mode == SOLENOID_MODE_ON)  digitalWrite(PIN_SOLENOID,HIGH);
  else if(mode == SOLENOID_MODE_OFF) digitalWrite(PIN_SOLENOID,LOW);
  else                               digitalWrite(PIN_SOLENOID,LOW);
}

//ros callback function
void callback(const std_msgs::Int32MultiArray& msg){
  m_pw      = msg.data[0];
  sol_mode  = msg.data[1];
  order_enc = msg.data[2];
}
