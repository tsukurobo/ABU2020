#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <pr_kpp_test_joycon/kick_msg.h>
#include <pr_kpp_test_joycon/pp_msg.h>
#include <sstream>

ros::Subscriber sub_joy;
ros::Publisher  pub_kick;
ros::Publisher  pub_pp;

pr_kpp_test_joycon::kick_msg data_kick;
pr_kpp_test_joycon::pp_msg   data_pp;

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
	pub_kick.publish(data_kick);
	pub_pp.publish(data_pp);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pr_kpp_joycon_node");
	ros::NodeHandle nh;
	
	sub_joy = nh.subscribe("joy", 1, get_joy);	
	pub_kick = nh.advertise<pr_kpp_test_joycon::kick_msg>("kick_tpc",1);
	pub_pp   = nh.advertise<pr_kpp_test_joycon::pp_msg>("pp_tpc",1);

	ros::Rate r(10.0);

	while(nh.ok()){
		ros::spinOnce();
		r.sleep();
	}
}
