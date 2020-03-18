#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <pr_msg/PpMsg.h>
#include <sstream>

//global variable
ros::Subscriber sub_beg; //subscriber from upper layer (tpc "pp_tpc")
ros::Subscriber sub_fin; //subscriber from arduino(tpc "pp_fin")
ros::Publisher  pub_beg; //publisher to arduino(tpc "pp_order")
ros::Publisher  pub_fin; //publisher to upper layer(tpc "pp_tpc")
int FREQ; //メインループ周波数[Hz]
int POW_LOWER; //手を下ろすモータパワー (-255~255)
int POW_RAISE; //手を上げるモータパワー (-255~255)
int POW_WIND;  //ロープ巻き上げモータパワー (-255~255)
int DEG_1; //ピックアップ時に動かす角度[deg]
int DEG_2; //ピックアップ時に戻す角度[deg]
int DELAY_SOL;  //ソレノイドonの時間[milli sec]
int DELAY_HAND; //ピックアップ時つかむ前後の待ち時間[milli sec]

//function protype
void cb_begin_task(const pr_msg::PpMsg& beg_order);
void cb_finish_task(const std_msgs::Int16MultiArray& fin_msg);

int main(int argc, char **argv){
	//node init
	ros::init(argc,argv,"pick_pass_node");
	ros::NodeHandle nh;

	sub_beg = nh.subscribe("pp_tpc",10,cb_begin_task);
	sub_fin = nh.subscribe("pp_fin",10,cb_finish_task);
	pub_beg = nh.advertise <std_msgs::Int16MultiArray>("pp_order",1);
	pub_fin = nh.advertise <pr_msg::PpMsg>("pp_tpc",1);

	//paramerter
	nh.getParam("pick_pass_node/FREQ",FREQ);
	nh.getParam("pick_pass_node/POW_LOWER",POW_LOWER);
	nh.getParam("pick_pass_node/POW_RAISE",POW_RAISE);
	nh.getParam("pick_pass_node/POW_WIND",POW_WIND);
	nh.getParam("pick_pass_node/DEG_1",DEG_1);
	nh.getParam("pick_pass_node/DEG_2",DEG_2);
	nh.getParam("pick_pass_node/DELAY_SOL",DELAY_SOL);
	nh.getParam("pick_pass_node/DELAY_HAND",DELAY_HAND);

	ros::Rate loop_rate(FREQ);

	//node body
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void cb_begin_task(const pr_msg::PpMsg& beg_order){
	std_msgs::Int16MultiArray data;
	data.data.resize(9);

	data.data[0] = beg_order.pick;
	data.data[1] = beg_order.launch;
	data.data[2] = POW_LOWER;
	data.data[3] = POW_RAISE;
	data.data[4] = POW_WIND;
	data.data[5] = DEG_1;
	data.data[6] = DEG_2;
	data.data[7] = DELAY_SOL;
	data.data[8] = DELAY_HAND;

	pub_beg.publish(data);
}

void cb_finish_task(const std_msgs::Int16MultiArray& fin_msg){
	pr_msg::PpMsg data;

	data.pick   = fin_msg.data[0];
	data.launch = fin_msg.data[1];

	pub_fin.publish(data);
}
