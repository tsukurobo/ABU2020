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
const uint8_t ADDR_TEE   = 0x23; //tee motor address of AVR
//pin setting
const int PIN_SW_RIGHT = 11;
const int PIN_SW_LEFT  = 8;
const int PIN_SW_HOLD  = 2;
const int PIN_SW_OPEN  = 5;
//others
const int MAIN_DELAY = 10; //[milli sec]
const int TOUCH_OFF  = LOW;

////global variable
///pamareter
int order_task = 0; //order of loading task
//motor power
int MOT_SLIDE_PW;
int MOT_RAISE_PW;
int MOT_LOWER_PW;
int MOT_GRASP_PW;
int MOT_TEE_PW;
//encoder
int ENC_LIFT_TOP;
int ENC_LIFT_MIDDLE;
int ENC_LIFT_BUTTOM;
int ENC_TEE_HOLD;
int ENC_TEE_OPEN;
///variable
long enc_lift = 0;
long enc_tee = 0;

//function prototype
void callback(const std_msgs::Int32MultiArray& msg);
void get_enc(const std_msgs::Int64MultiArray& msg);
void finish_task();
void all_stop();
void rtoo();
void hold_hold();
void open_open();
void go_right();
void go_left();
void go_top();
void go_buttom();
void go_middle();

//init
ros::NodeHandle nh;
std_msgs::Int32 data_fin;
ros::Publisher pub("losd_fin",&data_fin);
ros::Subscriber<std_msgs::Int32MultiArray> sub("load_order",&callback);
ros::Subscriber<std_msgs::Int64MultiArray> sub_enc("enc",&get_enc);

IseMotorDriver mot_slide(ADDR_SLIDE);
IseMotorDriver mot_lift(ADDR_LIFT);
IseMotorDriver mot_grasp(ADDR_GRASP);
IseMotorDriver mot_tee(ADDR_TEE);

void setup(){
  Wire.begin();

  //init node
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  nh.subscribe(sub_enc);
  
  order_task = 0;
  data_fin.data = 0;

  pinMode(PIN_SW_RIGHT,INPUT_PULLUP);
  pinMode(PIN_SW_LEFT,INPUT_PULLUP);
  pinMode(PIN_SW_HOLD,INPUT_PULLUP);
  pinMode(PIN_SW_OPEN,INPUT_PULLUP);

  //init actuator
  mot_slide.setSpeed(0);
  mot_lift.setSpeed(0);
  mot_grasp.setSpeed(0);
  mot_tee.setSpeed(0);  
}

void loop(){
  nh.spinOnce();
  
  switch(order_task){
    case TASK_NOTHING:
      all_stop();
      break;
    
    case TASK_STANDING_BY:
      rtoo();
      delay(2000);
      hold_hold();
      
      if(order_task>0) finish_task(); 
      break;

    case TASK_SETTING_1:
      go_right();
      go_buttom();
      open_open();
      go_top();
      go_left();
      hold_hold();
      
      if(order_task>0) finish_task(); 
      break;
      
    case TASK_SETTING_2:
      go_right();
      go_buttom();
      open_open();
      go_middle();
      go_left();
      hold_hold();
      
      if(order_task>0) finish_task(); 
      break;
      
    case TASK_SETTING_3:
      go_right();
      go_buttom();
      open_open();
      go_middle();
      go_left();
      
      if(order_task>0) finish_task(); 
      break;
      
    case EMERGENCY_STOP:
      all_stop();
      break;
    
  }
  delay(MAIN_DELAY);
}

void rtoo(){
  int slide_pw, lift_pw, grasp_pw, tee_pw;

  while(digitalRead(PIN_SW_RIGHT)==TOUCH_OFF || enc_lift<ENC_LIFT_TOP || digitalRead(PIN_SW_OPEN)==TOUCH_OFF || enc_tee<ENC_TEE_OPEN){
    nh.spinOnce();
    if(order_task < 0) break;
    
    //enc_lift = mot_lift.encorder();
    //enc_tee  = mot_tee.encorder();
    
    digitalRead(PIN_SW_RIGHT)==TOUCH_OFF ? slide_pw=-MOT_SLIDE_PW : slide_pw=0;
    enc_lift < ENC_LIFT_TOP              ?  lift_pw=MOT_RAISE_PW  :  lift_pw=0;
    digitalRead(PIN_SW_OPEN)==TOUCH_OFF  ? grasp_pw=-MOT_GRASP_PW : grasp_pw=0;
    enc_tee  < ENC_TEE_OPEN              ?   tee_pw=MOT_TEE_PW    :   tee_pw=0;   
    
    mot_slide.setSpeed(slide_pw);
    mot_lift.setSpeed(lift_pw);
    mot_grasp.setSpeed(grasp_pw);
    mot_tee.setSpeed(tee_pw); 

    delay(MAIN_DELAY);
  }

  all_stop();
}

void hold_hold(){
  int grasp_pw, tee_pw;

  while(digitalRead(PIN_SW_HOLD)==TOUCH_OFF || enc_tee>ENC_TEE_HOLD){
    nh.spinOnce();
    if(order_task < 0) break;
    
    //enc_tee = mot_tee.encorder();
    
    digitalRead(PIN_SW_HOLD)==TOUCH_OFF ? grasp_pw=MOT_GRASP_PW : grasp_pw=0;
    enc_tee > ENC_TEE_HOLD              ?   tee_pw=-MOT_TEE_PW  :   tee_pw=0;   
    
    mot_grasp.setSpeed(grasp_pw);
    mot_tee.setSpeed(tee_pw); 

    delay(MAIN_DELAY);
  }  
  mot_grasp.setSpeed(0);
  mot_tee.setSpeed(0);
}

void open_open(){
  int grasp_pw, tee_pw;

  while(digitalRead(PIN_SW_OPEN)==TOUCH_OFF || enc_tee<ENC_TEE_OPEN){
    nh.spinOnce();
    if(order_task < 0) break;
    
    //enc_tee  = mot_tee.encorder();
    
    digitalRead(PIN_SW_OPEN)==TOUCH_OFF  ? grasp_pw=-MOT_GRASP_PW : grasp_pw=0;
    enc_tee  < ENC_TEE_OPEN              ?   tee_pw=MOT_TEE_PW    :   tee_pw=0;   
    
    mot_grasp.setSpeed(grasp_pw);
    mot_tee.setSpeed(tee_pw); 

    delay(MAIN_DELAY);
  }  
  mot_grasp.setSpeed(0);
  mot_tee.setSpeed(0);
}

void go_right(){
  while(digitalRead(PIN_SW_RIGHT)==TOUCH_OFF){
    nh.spinOnce();
    if(order_task < 0) break;
    mot_slide.setSpeed(-MOT_SLIDE_PW);
    delay(MAIN_DELAY);
  }
  mot_slide.setSpeed(0);
}

void go_left(){
  while(digitalRead(PIN_SW_LEFT)==TOUCH_OFF){
    nh.spinOnce();
    if(order_task < 0) break;
    mot_slide.setSpeed(MOT_SLIDE_PW);
    delay(MAIN_DELAY);
  }
  mot_slide.setSpeed(0);
}

void go_top(){
  while(enc_lift<ENC_LIFT_TOP){
    nh.spinOnce();
    if(order_task < 0) break;
    mot_lift.setSpeed(MOT_RAISE_PW);
    delay(MAIN_DELAY);
  }
  mot_slide.setSpeed(0);
}

void go_buttom(){
  while(enc_lift>ENC_LIFT_BUTTOM){
    nh.spinOnce();
    if(order_task < 0) break;
    mot_lift.setSpeed(MOT_LOWER_PW);
    delay(MAIN_DELAY);
  }
  mot_slide.setSpeed(0);
}

void go_middle(){
  while(enc_lift<ENC_LIFT_MIDDLE){
    nh.spinOnce();
    if(order_task < 0) break;
    mot_lift.setSpeed(MOT_RAISE_PW);
    delay(MAIN_DELAY);
  }
  mot_slide.setSpeed(0);
}

void all_stop(){
  mot_slide.setSpeed(0);
  mot_lift.setSpeed(0);
  mot_grasp.setSpeed(0);
  mot_tee.setSpeed(0);
}
void finish_task(){
  data_fin.data = 0;
  pub.publish(&data_fin);
}

//ros callback function
void callback(const std_msgs::Int32MultiArray& msg){
  order_task      = msg.data[0];
  MOT_SLIDE_PW    = msg.data[1];
  MOT_RAISE_PW    = msg.data[2];
  MOT_LOWER_PW    = msg.data[3];
  MOT_GRASP_PW    = msg.data[4];
  MOT_TEE_PW      = msg.data[5];
  ENC_LIFT_TOP    = msg.data[6];
  ENC_LIFT_MIDDLE = msg.data[7];
  ENC_LIFT_BUTTOM = msg.data[8];
  ENC_TEE_HOLD    = msg.data[9];
  ENC_TEE_OPEN    = msg.data[10];
}

void get_enc(const std_msgs::Int64MultiArray& msg){
  enc_lift = msg.data[0];
  enc_tee  = msg.data[1];
}
