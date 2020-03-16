#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include <sstream>

ros::Subscriber sub_joy;
ros::Publisher pub_kick;
ros::Publisher pub_pp;

std_msgs::Int16MultiArray data_kick; 
std_msgs::Int16MultiArray data_pp;

void get_joy(const sensor_msgs::Joy& joy){
	data_kick.data[0] = joy.buttons[5];
	data_kick.data[1] = joy.buttons[4];

	data_pp.data[0] = joy.buttons[1];
	data_pp.data[1] = joy.buttons[0];
	data_pp.data[2] = joy.buttons[3];

	pub_kick.publish(data_kick);
	pub_pp.publish(data_pp);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pr_kpp_joycon_node");
	ros::NodeHandle nh;
	
	sub_joy = nh.subscribe("joy", 1, get_joy);	
	pub_kick = nh.advertise<std_msgs::Int16MultiArray>("kick_order",1);
	pub_pp   = nh.advertise<std_msgs::Int16MultiArray>("pp_order",1);

	ros::Rate r(10.0);

	data_kick.data.resize(2);
	data_pp.data.resize(3);

	while(nh.ok()){
		ros::spinOnce();
		r.sleep();
	}
}
