#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <sstream>

//global variable
ros::Subscriber sub_beg; //subscriber from upper layer (tpc"load_tpc")
ros::Subscriber sub_fin; //subscriber from arduino (tpc"load_fin")
ros::Publisher  pub_beg; //publisher to arduino (tpc"load_order")
ros::Publisher  pub_fin; //publisher to upper layer (tpc"load_tpc")
//ros::Publisher  pub_param;

int FREQ = 100;  //frequency of main loop [Hz]
int MOT_SLIDE_PW;
int MOT_RAISE_PW;
int MOT_LOWER_PW;
int MOT_GRASP_PW;
int MOT_TEE_PW;
int ENC_LIFT_MIDDLE;
int DELAY_HOLD;
int DELAY_SET;

//function protype
void cb_begin_task(const std_msgs::Int32& beg_order);
void cb_finish_task(const std_msgs::Int32& fin_msg);

int main(int argc, char **argv){
	int count = 0;
	
	//node init
	ros::init(argc,argv,"load_ball_node");
	ros::NodeHandle nh;

	sub_beg = nh.subscribe("load_tpc",10,cb_begin_task);
	sub_fin = nh.subscribe("load_fin",10,cb_finish_task); 
	pub_beg = nh.advertise <std_msgs::Int32MultiArray>("load_order",1);
	pub_fin = nh.advertise <std_msgs::Int32>("load_tpc",1);
	//pub_param = nh.advertise <std_msgs::Int32MultiArray>("load_param",1);

	//parameter
	//std_msgs::Int32MultiArray data_param;
	nh.getParam("load_ball_node/MOT_SLIDE_PW",MOT_SLIDE_PW);
	nh.getParam("load_ball_node/MOT_RAISE_PW",MOT_RAISE_PW);
	nh.getParam("load_ball_node/MOT_LOWER_PW",MOT_LOWER_PW);
	nh.getParam("load_ball_node/MOT_GRASP_PW",MOT_GRASP_PW);
	nh.getParam("load_ball_node/ENC_LIFT_MIDDLE",ENC_LIFT_MIDDLE);
	nh.getParam("load_ball_node/DELAY_HOLD",DELAY_HOLD);
	nh.getParam("load_ball_node/DELAY_SET",DELAY_SET);

//	data_param.data.resize(10);
//	data_param.data[0] = MOT_SLIDE_PW;
//	data_param.data[1] = MOT_RAISE_PW;
//	data_param.data[2] = MOT_LOWER_PW;
//	data_param.data[3] = MOT_GRASP_PW;
//	data_param.data[4] = MOT_TEE_PW;
//	data_param.data[5] = ENC_LIFT_TOP;
//	data_param.data[6] = ENC_LIFT_MIDDLE;
//	data_param.data[7] = ENC_LIFT_BUTTOM;
//	data_param.data[8] = ENC_TEE_HOLD;
//	data_param.data[9] = ENC_TEE_OPEN;

	ros::Rate loop_rate(FREQ);

	//node body
	while(ros::ok()){
		ros::spinOnce();
		//pub_param.publish(data_param);
		loop_rate.sleep();
	}

	return 0;
}

void cb_begin_task(const std_msgs::Int32& beg_order){
	std_msgs::Int32MultiArray data;
	data.data.resize(8);

	data.data[0] = beg_order.data;
	data.data[1] = MOT_SLIDE_PW;
	data.data[2] = MOT_RAISE_PW;
	data.data[3] = MOT_LOWER_PW;
	data.data[4] = MOT_GRASP_PW;
	data.data[5] = ENC_LIFT_MIDDLE;
	data.data[6] = DELAY_HOLD;
	data.data[7] = DELAY_SET;
	

	pub_beg.publish(data);
}

void cb_finish_task(const std_msgs::Int32& fin_msg){
	std_msgs::Int32 data;

	data.data = fin_msg.data;
	
	pub_fin.publish(data);
}

