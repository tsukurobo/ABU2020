#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32.h>
#include <sstream>

//global variable
ros::Subscriber sub_beg; //subscriber from upper layer (tpc"load_tpc")
ros::Subscriber sub_fin; //subscriber from arduino (tpc"load_fin")
ros::Publisher  pub_beg; //publisher to arduino (tpc"load_order")
ros::Publisher  pub_fin; //publisher to upper layer (tpc"load_tpc")
//int POW;   //power of motor (-255~255)
//int FREQ;  //frequency of main loop [Hz]
//int DELAY; //delay time of solenoid on/off [milli sec]

//function protype

int main(int argc, char **argv){
	//node init
	ros::init(argc,argv,"load_ball_node");
	ros::NodeHandle nh;

	sub_beg = nh.subscribe("load_tpc",10,cb_begin_task);
	sub_fin = nh.subscribe("load_fin",10,cb_finish_task); 
	pub_beg = nh.advertise <std_msgs::Int16MultiArray>("load_order",1);
	pub_fin = nh.advertise <std_msgs::Int32>("load_tpc",1);

	//parameter
//	nh.getParam("kick_node/FREQ",FREQ);
//	nh.getParam("kick_node/POW",POW);
//	nh.getParam("kick_node/DELAY",DELAY);

	ros::Rate loop_rate(FREQ);

	//node body
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void cb_begin_task(const std_msg::Int32& beg_order){
	std_msgs::Int16MultiArray data;
	data.data.resize(4);

	data.data[0] = beg_order.data;
	//data.data[1] = beg_order.launch;
	//data.data[2] = POW;
	//data.data[3] = DELAY;

	pub_beg.publish(data);
}

void cb_finish_task(const std_msgs::Int32& fin_msg){
	std_msgs::Int32 data;

	data.data = fin_msg.data;
	
	pub_fin.publish(data);
}
