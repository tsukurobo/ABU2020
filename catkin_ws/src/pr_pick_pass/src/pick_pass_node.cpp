#include"ros/ros.h"
#include"std_msgs/Int16MultiArray.h"
#include<sstream>

//constant
const int MAIN_FREQUENCY = 10; //メインループ周波数[Hz]
const int POWER_WIND  = 60; //ロープ巻き上げモータパワー (-255~255)
const int POWER_LOWER = 60; //手を下ろすモータパワー (-255~255)
const int POWER_RAISE = 60; //手を上げるモータパワー (-255~255)
const int TARGET_DEG  = 120; //ピックアップ時に動かす角度[deg]

//global variable
ros::Subscriber sub; //subscriber from topic "pp_order"
ros::Publisher  pub; //publisher to topic "pp_tpc"
std_msgs::Int16MultiArray subData; //sub data from topic "pp_order"
std_msgs::Int16MultiArray pubData; //pub data to topic "pp_tpc"

//function protype
void callback(const std_msgs::Int16MultiArray& msg);

int main(int argc, char **argv){
	//variable declaration
	subData.data.resize(3);
	pubData.data.resize(7);
	pubData.data[3] = POWER_WIND;
	pubData.data[4] = POWER_LOWER;
	pubData.data[5] = POWER_RAISE;
	pubData.data[6] = TARGET_DEG;

	//node init
	ros::init(argc,argv,"pick_pass_node");
	ros::NodeHandle nh;

	sub = nh.subscribe("pp_order",10,callback);
	pub = nh.advertise <std_msgs::Int16MultiArray>("pp_tpc",1);

	ros::Rate loop_rate(MAIN_FREQUENCY);

	//node body
	while(ros::ok()){
		ros::spinOnce();
		pub.publish(pubData);
		loop_rate.sleep();
	}

	return 0;
}

void callback(const std_msgs::Int16MultiArray& msg){
	pubData.data[0] = msg.data[0];
	pubData.data[1] = msg.data[1];
	pubData.data[2] = msg.data[2];
}
