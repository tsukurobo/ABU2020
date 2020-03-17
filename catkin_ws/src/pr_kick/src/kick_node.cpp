#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <pr_kpp_test_joycon/kick_msg.h>
#include <sstream>

//global variable
ros::Subscriber sub_beg; //subscriber from upper layer (tpc"kick_tpc")
ros::Subscriber sub_fin; //subscriber from arduino (tpc"kick_fin")
ros::Publisher  pub_beg; //publisher to arduino (tpc"kick_order")
ros::Publisher  pub_fin; //publisher to upper layer (tpc"kick_tpc")
int POW;   //power of motor (-255~255)
int FREQ;  //frequency of main loop [Hz]
int DELAY; //delay time of solenoid on/off [milli sec]

//function protype
void cb_begin_task(const pr_kpp_test_joycon::kick_msg& beg_order);
void cb_finish_task(const std_msgs::Int16MultiArray& fin_msg);

int main(int argc, char **argv){
	//node init
	ros::init(argc,argv,"kick_node");
	ros::NodeHandle nh;

	sub_beg = nh.subscribe("kick_tpc",10,cb_begin_task);
	sub_fin = nh.subscribe("kick_fin",10,cb_finish_task); 
	pub_beg = nh.advertise <std_msgs::Int16MultiArray>("kick_order",1);
	pub_fin = nh.advertise <pr_kpp_test_joycon::kick_msg>("kick_tpc",1);

	//parameter
	nh.getParam("kick_node/FREQ",FREQ);
	nh.getParam("kick_node/POW",POW);
	nh.getParam("kick_node/DELAY",DELAY);

	ros::Rate loop_rate(FREQ);

	//node body
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void cb_begin_task(const pr_kpp_test_joycon::kick_msg& beg_order){
	std_msgs::Int16MultiArray data;
	data.data.resize(4);

	data.data[0] = beg_order.wind;
	data.data[1] = beg_order.launch;
	data.data[2] = POW;
	data.data[3] = DELAY;

	pub_beg.publish(data);
}

void cb_finish_task(const std_msgs::Int16MultiArray& fin_msg){
	pr_kpp_test_joycon::kick_msg data;

	data.wind   = fin_msg.data[0];
	data.launch = fin_msg.data[1];
	
	pub_fin.publish(data);
}
