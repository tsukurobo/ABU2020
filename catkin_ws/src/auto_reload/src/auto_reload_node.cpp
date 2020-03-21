#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#define OPEN 1
#define CLOSE 0
#define TOP 1
#define MIDDLE 0
#define BUTTOM -1
#define KICK_SIDE -1
#define RACK_SIDE 1
#define FINISH 1
#define INIT 0
#define FIRST_KICK 1
#define SECOND_KICK 2
#define THIRD_KICK 3
#define NONE 4

ros::Subscriber joy_sub;
ros::Subscriber fin_sub;
ros::Publisher task_pub;

int fin_cmd = 0;
int task_num = 0;
int move_num = 0;
std_msgs::Int16MultiArray move_cmd;

void grasp(int status);
void elevate(int status);
void slide(int status);
void tee_hold(int status);





void get_cmd(const std_msgs::Int16& cmd){
	
    task_num = cmd.data;

}

void get_fin(const std_msgs::Int16::ConstPtr& fin_state){

    fin_cmd = fin_state->data;
    if(fin_cmd == FINISH){

        for(int i ;i < 5; i++){
            move_cmd.data[i] = 0;
        }
        move_num++;

    }
	
}



int main(int argc,char **argv){
	ros::init(argc,argv,"auto_reload_node");
	ros::NodeHandle nh;

	joy_sub = nh.subscribe("joy",1,get_cmd);
    fin_sub = nh.subscribe("task_cmd",1,get_fin);
	task_pub = nh.advertise<std_msgs::Int16MultiArray>("task_cmd",1);

    std_msgs::Int16MultiArray move_cmd;
    move_cmd.data.resize(5);

    ros::Rate loop_rate(10);


    while(ros::ok()){

        ros::spinOnce();


        if(task_num = INIT){

            if (move_num == 0){
                grasp(OPEN);
            }else if (move_num == 1)
            {
                slide(KICK_SIDE);
            }else if (move_num == 2)
            {
                elevate(BUTTOM);
            }else if (move_num == 3)
            {
                tee_hold(OPEN);
                move_num = 0;
                task_num = NONE;
            }



        }else if(task_num == FIRST_KICK){

            if (move_num == 0)
            {
                grasp(OPEN);
            }else if (move_num == 1)
            {
                elevate(TOP);
                
            }else if (move_num == 2)
            {
                slide(RACK_SIDE);
            }else if (move_num == 3)
            {
                tee_hold(OPEN);

            }else if(move_num == 4)
            {

                grasp(CLOSE);
                move_num = 0;
                task_num = NONE;
            }

        }else if(task_num == SECOND_KICK){

            if (move_num == 0)
            {
                tee_hold(CLOSE);
            }else if (move_num == 1)
            {
                slide(KICK_SIDE);
                
            }else if (move_num == 2)
            {
                elevate(BUTTOM);

            }else if (move_num == 3)
            {
                grasp(OPEN);

            }else if (move_num == 4)
            {
                elevate(MIDDLE);

            }else if (move_num == 5)
            {
                slide(RACK_SIDE);
                
            }if (move_num == 6)
            {
                tee_hold(OPEN);

            }else if(move_num == 7)
            {

                grasp(CLOSE);
                move_num = 0;
                task_num = NONE;
            }



        }else if(task_num == THIRD_KICK){

            if (move_num == 0)
            {
                tee_hold(CLOSE);
            }else if (move_num == 1)
            {
                slide(KICK_SIDE);
                
            }else if (move_num == 2)
            {
                elevate(BUTTOM);

            }else if (move_num == 3)
            {
                grasp(OPEN);

            }else if (move_num == 4)
            {
                elevate(MIDDLE);

            }else if (move_num == 5)
            {
                slide(RACK_SIDE);
                
            }if (move_num == 6)
            {
                tee_hold(OPEN);

            }else if(move_num == 7)
            {

                move_num = 0;
                task_num = NONE;
            }

            

            


        }

        ros::spinOnce();
        loop_rate.sleep();


    }

	return 0;
}

void grasp( int status){

    move_cmd.data[0] = status;
    task_pub.publish(move_cmd);
}

void elevate( int status){

    move_cmd.data[1] = status;
    task_pub.publish(move_cmd);
}

void slide( int status){

    move_cmd.data[2] = status;
    task_pub.publish(move_cmd);
}

void tee_hold( int status){

    move_cmd.data[3] = status;
    task_pub.publish(move_cmd);
}






