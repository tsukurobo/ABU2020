#include"ros/ros.h"
#include"std_msgs/Int16MultiArray.h"
#include<sstream>

//constant
const int MAIN_FREQUENCY = 10; //メインループ周波数[Hz]
const int POWER = -60; //モータパワー (0~255)

//global variable
ros::Subscriber sub; //subscriber from topic "kick_order"
ros::Publisher  pub; //publisher to topic "kick_tpc"
std_msgs::Int16MultiArray subData; //sub data from topic "kick_order"
std_msgs::Int16MultiArray pubData; //pub data to topic "kick_tpc"
bool wind_order; //winding order from joycon
bool kick_order; //kicking order from jycon

//function protype
void callback(const std_msgs::Int16MultiArray& msg);

int main(int argc, char **argv){
	//variable declaration
	subData.data.resize(2);
	pubData.data.resize(3);
	pubData.data[2] = POWER;

	//node init
	ros::init(argc,argv,"kick_node");
	ros::NodeHandle nh;

	sub = nh.subscribe("kick_order",10,callback);
	pub = nh.advertise <std_msgs::Int16MultiArray>("kick_tpc",1000);

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
}
