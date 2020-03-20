#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <pr_msg/KickMsg.h>
#include <pr_msg/PpMsg.h>
#include <sstream>

ros::Subscriber sub_joy;
ros::Publisher  pub_kick;
ros::Publisher  pub_pp;

pr_msg::KickMsg data_kick;
pr_msg::PpMsg   data_pp;

void get_joy(const sensor_msgs::Joy& joy){
	data_kick.wind   = joy.buttons[5];
	data_kick.launch = joy.buttons[4];

	data_pp.pick   = joy.buttons[1];
	data_pp.launch = joy.buttons[0];

	//emergency stop
	if(joy.buttons[2]==1){
		data_kick.wind   = -1;
		data_kick.launch = -1;
		data_pp.pick     = -1;
		data_pp.launch   = -1;
	}

	if(!(data_kick.wind==0 && data_kick.launch==0)) pub_kick.publish(data_kick);
	if(!(data_pp.pick==0 && data_pp.launch==0)) pub_pp.publish(data_pp);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pr_kpp_joycon_node");
	ros::NodeHandle nh;
	
	sub_joy = nh.subscribe("joy", 1, get_joy);	
	pub_kick = nh.advertise<pr_msg::KickMsg>("kick_tpc",1);
	pub_pp   = nh.advertise<pr_msg::PpMsg>("pp_tpc",1);

	ros::Rate r(10.0);

	while(nh.ok()){
		ros::spinOnce();
		r.sleep();
	}
}
