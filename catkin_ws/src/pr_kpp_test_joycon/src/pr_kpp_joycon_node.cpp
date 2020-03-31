#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
#include <pr_msg/KickMsg.h>
#include <pr_msg/PpMsg.h>
#include <sstream>

ros::Subscriber sub_joy;
ros::Subscriber sub_pp;
ros::Publisher  pub_kick;
ros::Publisher  pub_pp;
ros::Publisher  pub_load;

pr_msg::KickMsg data_kick;
pr_msg::PpMsg   data_pp;
std_msgs::Int32 data_load;

int old;
int old_pick;
int old_launch;
int old_reverse;

void get_joy(const sensor_msgs::Joy& joy){
	data_kick.wind   = joy.buttons[5];
	data_kick.launch = joy.buttons[4];

	//if(!(joy.buttons[1]==0 && data_pp.pick==1))    data_pp.pick    = joy.buttons[1];
	//if(!(joy.buttons[0]==0 && data_pp.launch==1))  data_pp.launch  = joy.buttons[0];
	//if(!(joy.buttons[3]==0 && data_pp.reverse==1)) data_pp.reverse = joy.buttons[3];

	if(joy.buttons[1]==1) data_pp.pick = 1;
	if(joy.buttons[0]==1) data_pp.launch = 1;
	if(joy.buttons[3]==1) data_pp.reverse = 1;

	if(joy.buttons[16]==1) data_load.data = 1;
	if(joy.buttons[14]==1) data_load.data = 2;
	if(joy.buttons[15]==1) data_load.data = 3;
	if(joy.buttons[13]==1) data_load.data = 4;

	//emergency stop
	if(joy.buttons[2]==1){
		data_kick.wind   = -1;
		data_kick.launch = -1;
		data_pp.pick     = -1;
		data_pp.launch   = -1;
		data_pp.reverse  = -1;
		data_load.data   = -1;
	}
	
	if(!(data_kick.wind==0 && data_kick.launch==0)) pub_kick.publish(data_kick);
	if(!(data_pp.pick==0 && data_pp.launch==0 && data_pp.reverse==0)) pub_pp.publish(data_pp);
	if(!(data_load.data==0 || old==data_load.data)) pub_load.publish(data_load);

	old = data_load.data;
	old_pick = data_pp.pick;
	old_launch = data_pp.launch;
	old_reverse = data_pp.reverse;
}

void get_pp_tpc(const pr_msg::PpMsg& pp){
	if(old_pick==1 && pp.pick==0) data_pp.pick = 0;
	if(old_launch==1 && pp.launch==0) data_pp.launch = 0;
	if(old_reverse==1 && pp.reverse==0) data_pp.reverse = 0;

	old_pick = data_pp.pick;
	old_launch = data_pp.launch;
	old_reverse = data_pp.reverse;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pr_kpp_joycon_node");
	ros::NodeHandle nh;
	
	sub_joy = nh.subscribe("joy", 1, get_joy);	
	sub_pp  = nh.subscribe("pp_tpc", 1, get_pp_tpc);	
	pub_kick = nh.advertise<pr_msg::KickMsg>("kick_tpc",1);
	pub_pp   = nh.advertise<pr_msg::PpMsg>("pp_tpc",1);
	pub_load = nh.advertise<std_msgs::Int32>("load_tpc",1);

	ros::Rate r(10.0);

	while(nh.ok()){
		ros::spinOnce();
		r.sleep();
	}
}
