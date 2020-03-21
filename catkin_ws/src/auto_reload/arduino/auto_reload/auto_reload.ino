#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#define RACK_SW 1
#define KICK_SW 2
#define CATCH_SW 3
#define RELEASE_SW 4

#define TOP 1000
#define MIDDLE 500
#define BUTTOM 0

#define TATCH 0

#define OPEN 1
#define CLOSE 0

#define TOP 1
#define MIDDLE 0
#define BUTTON -1

#define KICK_SIDE -1
#define RACK_SIDE 1

#define POW 100



ros::NodeHandle  nh;
std_msgs::Int16MultiArray cmd;

void get_cmd(const std_msgs::Int16MultiArray& now_cmd){
  
  cmd = now_cmd;
  
  
  }

ros::Publisher task_fin("task_cmd", &cmd);
ros::Subscriber<std_msgs::Int16MultiArray>task_sub("task_cmd",get_cmd);

IseMotorDriver grasp_motor = IseMotorDriver(0x20);//20
IseMotorDriver slide_motor = IseMotorDriver(0x21);//21
IseMotorDriver elevate_motor = IseMotorDriver(0x22);//22
IseMotorDriver tee_hold_motor = IseMotorDriver(0x23);//23


void setup()
{

  Wire.begin();
  
  nh.initNode();
  nh.advertise(task_fin);
  nh.subscribe(task_sub);
  cmd.data_length = 5;
  cmd.data = (int*)malloc(sizeof(int)*5);

  pinMode(RACK_SW,INPUT_PULLUP);
  pinMode(KICK_SW,INPUT_PULLUP);
  pinMode(CATCH_SW,INPUT_PULLUP);
  pinMode(RELEASE_SW,INPUT_PULLUP);

}

void loop()
{
  
  nh.spinOnce();
  delay(500);
}

grasp(int state){
  
  if(state == OPEN){

    if(analogRead(RELEASE_SW) == TOUCH){
      
      grasp_motor.setSpeed(0);
       
    }

    else if(analogRead(RELEASE_SW) == TOUCH){

        cmd.data[4] = 1;
        grasp_motor.setSpeed(0);
        
      
      }

    

    
  }
  
  
}
