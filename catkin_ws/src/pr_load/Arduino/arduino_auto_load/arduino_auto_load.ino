//スライドモータは右が初期位置
//リフトエンコーダは1番上が初期角度0になるように
//グラスプモータは開いてるときが初期位置
//ティーエンコーダは閉じてるとき初期角度が0になるように
//各モータは初期位置から動く方のパワーを正とする

//リフトエンコーダは下がるほど値が負に大きく
//ティーエンコーダは開くほど値は正に大きく
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int64MultiArray.h>

////constant
//task
const int TASK_NOTHING     = 0;
const int TASK_STANDING_BY = 1;
const int TASK_SETTING_1   = 2;
const int TASK_SETTING_2   = 3;
const int TASK_SETTING_3   = 4;
const int EMERGENCY_STOP   = -1;
//AVR address
const uint8_t ADDR_SLIDE = 0x20; //slide motor address of AVR
const uint8_t ADDR_LIFT  = 0x21; //lift move motor address of AVR
const uint8_t ADDR_GRASP = 0x22; //grasp motor address of AVR
//const uint8_t ADDR_TEE   = 0x23; //tee motor address of AVR
//pin setting
const int PIN_SW_RIGHT  = 11;
const int PIN_SW_LEFT   = 8;
const int PIN_SW_HOLD   = 2;
const int PIN_SW_OPEN   = 5;
const int PIN_SW_TOP    = 4;
const int PIN_SW_BUTTOM = 9;
const int PIN_VALV_1    = 6;
const int PIN_VALV_2    = 10;
const int PIN_RACK_UP_1   = 57; //A3
const int PIN_RACK_UP_2   = 61; //A7
const int PIN_RACK_DOWN_1 = 64; //A11
const int PIN_RACK_DOWN_2 = 67; //A14

//others
const int MAIN_DELAY = 10; //[milli sec]
const int TOUCH_OFF  = LOW;
const int VALVE_OFF  = 0;
const int VALVE_HOLD = 2;
const int VALVE_OPEN = 1;

////global variable
///pamareter
int order_task = 0; //order of loading task
//motor power
int MOT_SLIDE_PW;
int MOT_RAISE_PW;
int MOT_LOWER_PW;
int MOT_GRASP_PW;
//int MOT_TEE_PW;
//encoder
int ENC_LIFT_MIDDLE;
//delay
int DELAY_HOLD;
int DELAY_SET;

///variable
int pw_slide = 0;
int pw_lift  = 0;
int pw_grasp = 0;
//int pw_tee   = 0;
long enc_lift = 0;

//function prototype
void callback(const std_msgs::Int32MultiArray& msg);
void get_enc(const std_msgs::Int64MultiArray& msg);
//void get_param(const std_msgs::Int32MultiArray& msg);

//init
ros::NodeHandle nh;
std_msgs::Int32 data_fin;
ros::Publisher pub("load_fin",&data_fin);
ros::Subscriber<std_msgs::Int32MultiArray> sub("load_order",&callback);
ros::Subscriber<std_msgs::Int64MultiArray> sub_enc("enc",&get_enc);
//ros::Subscriber<std_msgs::Int32MultiArray> sub_param("load_param",&get_param);
//debug
std_msgs::Int32 debug;
ros::Publisher pubD("debug_tpc",&debug);

IseMotorDriver mot_slide(ADDR_SLIDE);
IseMotorDriver mot_lift(ADDR_LIFT);
IseMotorDriver mot_grasp(ADDR_GRASP);
//IseMotorDriver mot_tee(ADDR_TEE);

void setup(){
  Wire.begin();

  //init node
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  nh.subscribe(sub_enc);
  //nh.subscribe(sub_param);
  //debug
  nh.advertise(pubD);
    
  order_task = 0;
  data_fin.data = 0;

  pinMode(PIN_SW_RIGHT,INPUT_PULLUP);
  pinMode(PIN_SW_LEFT,INPUT_PULLUP);
  pinMode(PIN_SW_HOLD,INPUT_PULLUP);
  pinMode(PIN_SW_OPEN,INPUT_PULLUP);
  pinMode(PIN_SW_TOP,INPUT_PULLUP);
  pinMode(PIN_SW_BUTTOM,INPUT_PULLUP);
  pinMode(PIN_VALV_1,OUTPUT);
  pinMode(PIN_VALV_2,OUTPUT);
  pinMode(PIN_RACK_UP_1,OUTPUT);
  pinMode(PIN_RACK_UP_2,OUTPUT);
  pinMode(PIN_RACK_DOWN_1,OUTPUT);
  pinMode(PIN_RACK_DOWN_2,OUTPUT);
  
  //init actuator
  all_stop();

}

void loop(){
  nh.spinOnce();
  
  switch(order_task){
    case TASK_NOTHING:
      all_stop();
      break;
      
    case TASK_STANDING_BY:
      go_right(); go_top(); grasp_open(); valve(VALVE_HOLD);
      go_buttom();
      grasp_hold_time(DELAY_HOLD);
      go_top();
      //valve(VALVE_HOLD);
      
      finish_task();
      break;  

    case TASK_SETTING_1:
      go_buttom();
      delay(DELAY_SET);
      grasp_open();
      go_top();
      go_left();
      grasp_hold_time(DELAY_HOLD);
      rack(1);
      valve(VALVE_OPEN);
      
      finish_task();
      break;

    case TASK_SETTING_2:
      valve(VALVE_HOLD);
      go_right();
      go_buttom();
      delay(DELAY_SET);
      grasp_open();
      go_middle();
      go_left();
      grasp_hold_time(DELAY_HOLD);
      rack(2);
      valve(VALVE_OPEN);
      
      finish_task();
      break;

    case TASK_SETTING_3:
      valve(VALVE_HOLD);
      go_right();
      go_buttom();
      delay(DELAY_SET);
      grasp_open();
      go_top();
      go_left();
      valve(VALVE_OPEN);
      
      finish_task();
      break;

    default:
      all_stop();
      break;
      
  }
  delay(MAIN_DELAY);
}

void go_right(){
  while(digitalRead(PIN_SW_RIGHT)==TOUCH_OFF){
    nh.spinOnce();
    if(order_task < 0) break;
    move_mot_slide(-MOT_SLIDE_PW);
    delay(MAIN_DELAY);
  }
  move_mot_slide(0);
}

void go_left(){
  while(digitalRead(PIN_SW_LEFT)==TOUCH_OFF){
    nh.spinOnce();
    if(order_task < 0) break;
    move_mot_slide(MOT_SLIDE_PW);
    delay(MAIN_DELAY);
  }
  move_mot_slide(0);
}

void go_top(){
  while(digitalRead(PIN_SW_TOP)==TOUCH_OFF){
    nh.spinOnce();
    if(order_task < 0) break;
    move_mot_lift(MOT_RAISE_PW);
    delay(MAIN_DELAY);
  }
  move_mot_lift(0);
}

void go_buttom(){
  while(digitalRead(PIN_SW_BUTTOM)==TOUCH_OFF){
    nh.spinOnce();
    if(order_task < 0) break;
    move_mot_lift(MOT_LOWER_PW);
    delay(MAIN_DELAY);
  }
  move_mot_lift(0);
}

void go_middle(){
  while(enc_lift<ENC_LIFT_MIDDLE){
    nh.spinOnce();
    if(order_task < 0) break;
    move_mot_lift(MOT_RAISE_PW);
    delay(MAIN_DELAY);
  }
  move_mot_lift(0);
}

void grasp_open(){
  while(digitalRead(PIN_SW_OPEN)==TOUCH_OFF){
    nh.spinOnce();
    if(order_task < 0) break;
    move_mot_grasp(MOT_GRASP_PW);
    delay(MAIN_DELAY);
  }
  move_mot_grasp(0);
}

void grasp_open_time(int delay_time){
  long tm;
  long beg;
  tm  = millis();
  beg = millis();
  while(tm-beg < delay_time){
    nh.spinOnce();
    if(order_task < 0) break;
    move_mot_grasp(MOT_GRASP_PW);
    delay(MAIN_DELAY);
    tm = millis();
  }
  move_mot_grasp(0);
}

void grasp_hold(){
  while(digitalRead(PIN_SW_HOLD)==TOUCH_OFF){
    nh.spinOnce();
    if(order_task < 0) break;
    move_mot_grasp(-MOT_GRASP_PW);
    delay(MAIN_DELAY);
  }
  move_mot_grasp(0);
}

void grasp_hold_time(int delay_time){
  long tm;
  long beg;
  tm  = millis();
  beg = millis();
  while(tm-beg < delay_time){
    nh.spinOnce();
    if(order_task < 0) break;
    move_mot_grasp(-MOT_GRASP_PW);
    delay(MAIN_DELAY);
    tm = millis();
  }
  move_mot_grasp(0);
}

//void rtoo(){
//  while(digitalRead(PIN_SW_RIGHT)==TOUCH_OFF || enc_lift<ENC_LIFT_TOP || digitalRead(PIN_SW_OPEN)==TOUCH_OFF || enc_tee<ENC_TEE_OPEN){
//    nh.spinOnce();
//    if(order_task < 0) break;
//    
//    //enc_lift = mot_lift.encorder();
//    //enc_tee  = mot_tee.encorder();
//    
//    digitalRead(PIN_SW_RIGHT)==TOUCH_OFF ? pw_slide=-MOT_SLIDE_PW : pw_slide=0;
//    enc_lift < ENC_LIFT_TOP              ?  pw_lift=MOT_RAISE_PW  :  pw_lift=0;
//    digitalRead(PIN_SW_OPEN)==TOUCH_OFF  ? pw_grasp=-MOT_GRASP_PW : pw_grasp=0;
//    enc_tee  < ENC_TEE_OPEN              ?   pw_tee=MOT_TEE_PW    :   pw_tee=0;   
//    
//    mot_slide.setSpeed(pw_slide);
//    mot_lift.setSpeed(pw_lift);
//    mot_grasp.setSpeed(pw_grasp);
//    mot_tee.setSpeed(pw_tee); 
//    delay(MAIN_DELAY);
//  }
//}
//
//void hold_hold(){
//  while(digitalRead(PIN_SW_HOLD)==TOUCH_OFF || enc_tee>ENC_TEE_HOLD){
//    nh.spinOnce();
//    if(order_task < 0) break;
//    
//    //enc_tee = mot_tee.encorder();
//    
//    digitalRead(PIN_SW_HOLD)==TOUCH_OFF ? pw_grasp=MOT_GRASP_PW : pw_grasp=0;
//    enc_tee > ENC_TEE_HOLD              ?   pw_tee=-MOT_TEE_PW  :   pw_tee=0;   
//    
//    mot_grasp.setSpeed(pw_grasp);
//    nh.spinOnce();
//    mot_tee.setSpeed(pw_tee); 
//    nh.spinOnce();
//
//    delay(MAIN_DELAY);
//  }  
//  mot_grasp.setSpeed(0);
//  mot_tee.setSpeed(0);
//}

void move_mot_slide(int pw){
  if(pw_slide != pw){
      pw_slide = pw;
      mot_slide.setSpeed(pw_slide);
      nh.spinOnce();
    }
}

void move_mot_lift(int pw){
  if(pw_lift != pw){
      pw_lift = pw;
      mot_lift.setSpeed(pw_lift);
      nh.spinOnce();
    }
}

void move_mot_grasp(int pw){
  if(pw_grasp != pw){
      pw_grasp = pw;
      mot_grasp.setSpeed(pw_grasp);
      nh.spinOnce();
    }
}

void valve(int mode){
  if(mode == 1){
    digitalWrite(PIN_VALV_1,LOW);
    digitalWrite(PIN_VALV_2,LOW);
    digitalWrite(PIN_VALV_1,HIGH);
    digitalWrite(PIN_VALV_2,LOW);
    delay(1000);
  }else if(mode == 2){
    digitalWrite(PIN_VALV_1,LOW);
    digitalWrite(PIN_VALV_2,LOW);
    digitalWrite(PIN_VALV_1,LOW);
    digitalWrite(PIN_VALV_2,HIGH);
    delay(1000);
  }else{
    digitalWrite(PIN_VALV_1,LOW);
    digitalWrite(PIN_VALV_2,LOW);
  }
}

void rack(int mode){
  if(mode == 1){
    digitalWrite(PIN_RACK_UP_1,LOW);
    digitalWrite(PIN_RACK_UP_2,LOW);
    digitalWrite(PIN_RACK_UP_1,LOW);
    digitalWrite(PIN_RACK_UP_2,HIGH);
    delay(1000);
  }else if(mode == 2){
    digitalWrite(PIN_RACK_DOWN_1,LOW);
    digitalWrite(PIN_RACK_DOWN_2,LOW);
    digitalWrite(PIN_RACK_DOWN_1,HIGH);
    digitalWrite(PIN_RACK_DOWN_2,LOW);
    delay(1000);
  }else{
    digitalWrite(PIN_RACK_UP_1,LOW);
    digitalWrite(PIN_RACK_UP_2,LOW);
    digitalWrite(PIN_RACK_DOWN_1,LOW);
    digitalWrite(PIN_RACK_DOWN_2,LOW);
  }   
}

void all_stop(){
  if(!((pw_slide==0) && (pw_lift==0) && (pw_grasp==0))){
    pw_slide = 0;
    pw_lift  = 0;
    pw_grasp = 0;
    mot_slide.setSpeed(pw_slide); nh.spinOnce();
    mot_lift.setSpeed(pw_lift);   nh.spinOnce();
    mot_grasp.setSpeed(pw_grasp); nh.spinOnce();
  }
  digitalWrite(PIN_VALV_1,LOW);
  digitalWrite(PIN_VALV_2,LOW);
//  digitalWrite(PIN_RACK_UP_1,LOW);
//  digitalWrite(PIN_RACK_UP_2,LOW);
//  digitalWrite(PIN_RACK_DOWN_1,LOW);
//  digitalWrite(PIN_RACK_DOWN_2,LOW);
    digitalWrite(PIN_RACK_UP_1,HIGH);
  digitalWrite(PIN_RACK_UP_2,LOW);
  digitalWrite(PIN_RACK_DOWN_1,LOW);
  digitalWrite(PIN_RACK_DOWN_2,HIGH);
}

void finish_task(){
  if(order_task>0){
    data_fin.data = 0;
    pub.publish(&data_fin);
    order_task = 0; 
  }
}

//ros callback function
void callback(const std_msgs::Int32MultiArray& msg){
  order_task      = msg.data[0];
  MOT_SLIDE_PW    = msg.data[1];
  MOT_RAISE_PW    = msg.data[2];
  MOT_LOWER_PW    = msg.data[3];
  MOT_GRASP_PW    = msg.data[4];
  ENC_LIFT_MIDDLE = msg.data[5];
  DELAY_HOLD      = msg.data[6];
  DELAY_SET       = msg.data[7];
}

void get_enc(const std_msgs::Int64MultiArray& msg){
  enc_lift = msg.data[0];
  //enc_tee  = msg.data[1];
}
