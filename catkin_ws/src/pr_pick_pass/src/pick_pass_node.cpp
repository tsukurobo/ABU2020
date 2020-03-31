//LOW: lock on, HIGH: lock off
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <pr_msg/PpMsg.h>
#include <sstream>

//global variable
ros::Subscriber sub_order;  //subscriber from upper layer (tpc "pp_tpc")
ros::Subscriber sub_sensor; //subscriber from arduino(tpc "pp_sensor")
ros::Publisher  pub_order;  //publisher to arduino(tpc "pp_order")
ros::Publisher  pub_fin;    //publisher to upper layer(tpc "pp_tpc")
std_msgs::Int32MultiArray data_order;
pr_msg::PpMsg data_fin;

const int SOLENOID_MODE_ON = 1;
const int SOLENOID_MODE_OFF = 0;
const int SOLENOID_LOCK_OFF = 1;
const int SOLENOID_LOCK_ON = 0;
const int VALVE_MODE_OPEN = 1;
const int VALVE_MODE_HOLD = 2;
const int ENC_PER_ROT = 4048;

int order_pick;
int order_launch;
int order_reverse;
int touch_val;
long enc_pick;
long enc_pass;
int step_pick    = 1;
int step_launch  = 1;
int step_reverse = 1;

int FREQ = 100; //main loop frequency [Hz]
int POW_LOWER; //手を下ろすモータパワー (-255~255)
int POW_RAISE; //手を上げるモータパワー (-255~255)
int POW_WIND;  //ロープ巻き上げモータパワー (-255~255)
int DEG_1; //ピックアップ時に動かす角度[deg]
int DEG_2; //ピックアップ時に戻す角度[deg]
int DELAY_SOL;  //ソレノイドonの時間[milli sec]
int DELAY_HAND; //ピックアップ時つかむ前後の待ち時間[milli sec]
int DELAY_WIND; //巻取り時，タッチセンサが反応してから逆回転するまでの待ち時間[milli sec]
 
//function protype
void get_order(const pr_msg::PpMsg& msg_order);
void get_sensor(const std_msgs::Int32MultiArray& msg_sensor);
void task_pick();
void task_launch();
void task_reverse();
void all_stop();

int main(int argc, char **argv){
	//node init
	ros::init(argc,argv,"pick_pass_node");
	ros::NodeHandle nh;

	sub_order  = nh.subscribe("pp_tpc",1,get_order);
	sub_sensor = nh.subscribe("pp_sensor",1,get_sensor);
	pub_order  = nh.advertise <std_msgs::Int32MultiArray>("pp_order",1);
	pub_fin    = nh.advertise <pr_msg::PpMsg>("pp_tpc",1);

	//paramerter
	nh.getParam("pick_pass_node/FREQ",FREQ);
	nh.getParam("pick_pass_node/POW_LOWER",POW_LOWER);
	nh.getParam("pick_pass_node/POW_RAISE",POW_RAISE);
	nh.getParam("pick_pass_node/POW_WIND",POW_WIND);
	nh.getParam("pick_pass_node/DEG_1",DEG_1);
	nh.getParam("pick_pass_node/DEG_2",DEG_2);
	nh.getParam("pick_pass_node/DELAY_SOL",DELAY_SOL);
	nh.getParam("pick_pass_node/DELAY_HAND",DELAY_HAND);
	nh.getParam("pick_pass_node/DELAY_WIND",DELAY_WIND);

	ros::Rate loop_rate(FREQ);

	data_order.data.resize(4);
	data_order.data[0] = 0;
	data_order.data[1] = 0;
	data_order.data[2] = 0;
	data_order.data[3] = 0;

	//node body
	while(ros::ok()){
		ros::spinOnce();

		if(order_pick > 0) task_pick();
		if(order_launch > 0) task_launch();
		//if(order_reverse > 0) task_reverse();

		pub_order.publish(data_order);

		if(order_pick<0 || order_launch<0 || order_reverse<0) all_stop();
		loop_rate.sleep();
	}

	return 0;
}

void task_pick(){
	if(step_pick == 1){
		//open hand
		data_order.data[3] = VALVE_MODE_OPEN;
		//lower hand
		data_order.data[0] = POW_LOWER;
		//next
		if(DEG_1*ENC_PER_ROT/360.0 > enc_pick){
			data_order.data[0] = 0;
			step_pick = 2;
		}
	}else if(step_pick == 2){
		//wait for hold time
		static int cnt = 0;
		cnt++;
		//next
		if(cnt > DELAY_HAND*FREQ/1000.0){
			step_pick = 3;
			cnt = 0;
		}
	}else if(step_pick == 3){
		//hold hand
		data_order.data[3] = VALVE_MODE_HOLD;
		step_pick = 4;//next
	}else if(step_pick == 4){
		//wait for hold time
		static int cnt = 0;
		cnt++;
		//next
		if(cnt > DELAY_HAND*FREQ/1000.0){
			step_pick = 5;
			cnt = 0;
		}
	}else if(step_pick == 5){
		//raise hand
		data_order.data[0] = POW_RAISE;
		//next
		if(DEG_2*ENC_PER_ROT/360.0 < enc_pick){
			data_order.data[0] = 0;
			step_pick = 6;
		}
	}else if(step_pick == 6){
		order_pick = 0;
		step_pick = 1;
		data_fin.pick = 0;
		pub_fin.publish(data_fin);
	}
}

void task_launch(){
	if(step_launch == 1){
		//open hand
		data_order.data[3] = VALVE_MODE_OPEN;
		//lower hand
		data_order.data[0] = POW_LOWER;
		//next
		if(DEG_1*ENC_PER_ROT/360.0 > enc_pick){
			data_order.data[0] = 0;
			step_launch = 2;
		}
	}else if(step_launch == 2){
		//wait for hold time
		static int cnt = 0;
		cnt++;
		//next
		if(cnt > 500*FREQ/1000.0){
			step_launch = 3;
			cnt = 0;
		}
	}else if(step_launch == 3){
		//launch solenoid
		static int cnt = 0;
		
		if(cnt < DELAY_SOL*FREQ/1000.0){
			data_order.data[2] = SOLENOID_MODE_ON;
		}else if(cnt <DELAY_SOL*9.0*FREQ/1000.0){
			data_order.data[2] = SOLENOID_MODE_OFF;
		}else{
			cnt = 0;
		}
		cnt++;
		//next
		if(touch_val == SOLENOID_LOCK_OFF){
			data_order.data[2] = SOLENOID_MODE_OFF;
			step_launch = 4;
			cnt = 0;
		}
	}else if(step_launch == 4){
		//wait for hold time
		static int cnt = 0;
		cnt++;
		//next
		if(cnt > 1000.0*FREQ/1000.0){
			step_launch = 5;
			cnt = 0;
		}
	}else if(step_launch == 5){
		//wind rope
		data_order.data[1] = POW_WIND;
		//next
		if(touch_val == SOLENOID_LOCK_ON){
			data_order.data[1] = 0;
			step_launch = 6;
		}
	}else if(step_launch == 6){
		//raise hand
		data_order.data[0] = POW_RAISE;
		//next
		if(DEG_2*ENC_PER_ROT/360.0 < enc_pick){
			data_order.data[0] = 0;
			step_launch = 7;
		}
	}else if(step_launch == 7){
		//wind reverse rope
		data_order.data[1] = -POW_WIND;
		//next
		if(enc_pass > 0){
			data_order.data[1] = 0;
			step_launch = 8;
		}
	}else if(step_launch == 8){
		order_launch = 0;
		step_launch = 1;
		data_fin.launch = 0;
		pub_fin.publish(data_fin);
	}
}

void task_reverse(){
	if(step_reverse == 1){
		//raise hand
		data_order.data[0] = POW_RAISE;
		//next
		if(DEG_2*ENC_PER_ROT/360.0 < enc_pick){
			data_order.data[0] = 0;
			step_reverse = 2;
		}
	}else if(step_reverse == 2){
		order_pick = 0;
		step_reverse = 1;
		data_fin.reverse = 0;
		pub_fin.publish(data_fin);
	}
}

void all_stop(){
	data_order.data[0] = 0;
	data_order.data[1] = 0;
	data_order.data[2] = 0;
	data_order.data[3] = 0;
}

void get_order(const pr_msg::PpMsg& msg_order){
	order_pick    = msg_order.pick;
	order_launch  = msg_order.launch;
	order_reverse = msg_order.reverse;
}

void get_sensor(const std_msgs::Int32MultiArray& msg_sensor){
	touch_val = msg_sensor.data[0];
	enc_pick  = msg_sensor.data[1];
	enc_pass  = msg_sensor.data[2];
}
