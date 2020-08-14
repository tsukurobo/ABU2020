//LOW: lock on, HIGH: lock off
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <sstream>

//ros variable
ros::Subscriber sub_joy;    //subscribe from upper layer (tpc "joy")
ros::Subscriber sub_sensor; //subscribe from arduino(tpc "pp_sensor")
ros::Publisher  pub_order;  //publish to arduino(tpc "pp_order")
std_msgs::Int32MultiArray data_order;

//constant
const int FREQ = 100; //main loop frequency [Hz]
const int SOLENOID_MODE_ON = 1;
const int SOLENOID_MODE_OFF = 0;
const int SOLENOID_LOCK_OFF = 1;
const int SOLENOID_LOCK_ON = 0;
const int VALVE_MODE_OPEN = 1;
const int VALVE_MODE_HOLD = 2;
const int ENC_PER_ROT = 4048;
const int DEBUG_M_PW = 100;
const int ORDER_ENC_GET    = 1;
const int ORDER_ENC_NO_GET = 0;

//sub joy
//task order
int order_pick;
int order_launch;
int order_reverse;
int emg_stop;
//debug order
int debug_mode;
int debug_joy_LR;
int debug_joy_UD;
float debug_joy_stick;

//pub arduino
//actuator
int m_pw_pick;
int m_pw_pass;
int sol_mode;
int valve_mode;
//encoder order
int order_enc_pick;
int order_enc_pass;

//sub sensor
int touch_val;
long enc_pick;
long enc_pass;

//step
int step_pick    = 0;
int step_launch  = 0;
int step_reverse = 0;

//parameter
int POW_LOWER = 22; //手を下ろすモータパワー (-255~255)
int POW_RAISE; //手を上げるモータパワー (-255~255)
int POW_WIND;  //ロープ巻き上げモータパワー (-255~255)
int DEG_1; //ピックアップ時に動かす角度[deg]
int DEG_2; //ピックアップ時に戻す角度[deg]
int DELAY_SOL;  //ソレノイドonの時間[milli sec]
int DELAY_HAND; //ピックアップ時つかむ前後の待ち時間[milli sec]
int DELAY_WIND; //巻取り時，タッチセンサが反応してから逆回転するまでの待ち時間[milli sec]
 
//function protype
void get_joy(const sensor_msgs::Joy& msg_joy);
void get_sensor(const std_msgs::Int32MultiArray& msg_sensor);
void task_pick();
void task_launch();
void task_reverse();
void debug_func();
void all_stop();
void publish_order();

int main(int argc, char **argv){
	//node init
	ros::init(argc,argv,"new_pick_pass_node");
	ros::NodeHandle nh;

	sub_joy    = nh.subscribe("joy",1,get_joy);
	sub_sensor = nh.subscribe("pp_sensor",1,get_sensor);
	pub_order  = nh.advertise <std_msgs::Int32MultiArray>("pp_order",1);

	//paramerter
	nh.getParam("new_pick_pass_node/POW_LOWER",POW_LOWER);
	nh.getParam("new_pick_pass_node/POW_RAISE",POW_RAISE);
	nh.getParam("new_pick_pass_node/POW_WIND",POW_WIND);
	nh.getParam("new_pick_pass_node/DEG_1",DEG_1);
	nh.getParam("new_pick_pass_node/DEG_2",DEG_2);
	nh.getParam("new_pick_pass_node/DELAY_SOL",DELAY_SOL);
	nh.getParam("new_pick_pass_node/DELAY_HAND",DELAY_HAND);
	nh.getParam("new_pick_pass_node/DELAY_WIND",DELAY_WIND);

	ros::Rate loop_rate(FREQ);

	data_order.data.resize(6);
	data_order.data[0] = 0;
	data_order.data[1] = 0;
	data_order.data[2] = 0;
	data_order.data[3] = 0;
	data_order.data[4] = 0;
	data_order.data[5] = 0;

	//node body
	while(ros::ok()){
		ros::spinOnce();

		//mode change
		if(emg_stop == 1){ //emg_stop mode
			all_stop();
			publish_order();
		}else if(debug_mode == 1){ //debug mode
			debug_func();
			publish_order();
		}else{
			if(order_pick   == 1 || step_pick > 0)   task_pick();
			if(order_launch == 1 || step_launch > 0) task_launch();
		}



		loop_rate.sleep();
	}

	return 0;
}

void task_pick(){
	if(step_pick == 0){
		step_pick = 1;
	}else if (step_pick == 1) {
		valve_mode = VALVE_MODE_OPEN; //open hand
		m_pw_pick = POW_LOWER;        //lower hand
		order_enc_pick = ORDER_ENC_GET;

		if(DEG_1*ENC_PER_ROT/360.0 > enc_pick){ //stop hand
			m_pw_pick = 0;
			order_enc_pick = ORDER_ENC_NO_GET;
			step_pick = 2;
		}
	}else if(step_pick == 2){ //wait for hold hand
		static int cnt = 0;
		cnt++;
		
		if(cnt > DELAY_HAND*FREQ/1000){
			step_pick = 3;
			cnt = 0;
		}
	}else if(step_pick == 3){
		valve_mode = VALVE_MODE_HOLD; //hold hand
		m_pw_pick = POW_RAISE;        //raise hand
		order_enc_pick = ORDER_ENC_GET;

		if(DEG_2*ENC_PER_ROT/360.0 < enc_pick){ //stop hand
			m_pw_pick = 0;
			order_enc_pick = ORDER_ENC_NO_GET;
			step_pick = 0;
		}
	}	

	publish_order();
}

void task_launch(){
	if(step_launch == 0){
		valve_mode = VALVE_MODE_OPEN; //open hand
		step_launch = 1;
	}else if (step_launch == 1) { //wait for hand
		static int cnt = 0;
		cnt++;
		
		if(cnt > DELAY_HAND*FREQ/1000){
			step_launch = 2;
			cnt = 0;
		}
	}else if(step_launch == 2){
		m_pw_pick = POW_LOWER; //lower hand
		order_enc_pick = ORDER_ENC_GET;

		if(DEG_1*ENC_PER_ROT/360.0*2/3 > enc_pick){ //stop hand (本来より2/3位の角度で止まる)
			m_pw_pick = 0;
			order_enc_pick = ORDER_ENC_NO_GET;
			step_launch = 3;
		}
	}else if(step_launch == 3){
		static int cnt = 0;
		cnt++;

		if(touch_val == SOLENOID_LOCK_ON){ //launch ball
			if(DELAY_SOL*FREQ/1000 > cnt) sol_mode = SOLENOID_MODE_ON;
			else if(DELAY_SOL*FREQ*9/1000 > cnt) sol_mode = SOLENOID_MODE_OFF;
			else cnt = 0;
		}else{
			cnt = 0;
			step_launch = 4;
		}
	}else if(step_launch == 4){ //wait for launch time (200msec)
		static int cnt = 0;
		cnt++;

		if(cnt > 800*FREQ/1000){
			cnt = 0;
			step_launch = 5;
		}
	}else if(step_launch == 5){ //wind rope
		m_pw_pass = POW_WIND;

		if(touch_val == SOLENOID_LOCK_ON){
			step_launch = 6;
		}
	}else if(step_launch == 6){ //wait for wind
		static int cnt = 0;
		cnt++;

		if(cnt > DELAY_WIND*FREQ/1000){
			m_pw_pass = 0;
			cnt = 0;
			step_launch = 7;
		}
	}else if(step_launch == 7){ //raise hand
		m_pw_pick = POW_RAISE;
		order_enc_pick = ORDER_ENC_GET;

		if(DEG_2*ENC_PER_ROT/360.0 < enc_pick){ //stop hand
			m_pw_pick = 0;
			order_enc_pick = ORDER_ENC_NO_GET;
			step_launch = 8;
		}
	}else if(step_launch == 8){ //reverse rope
		m_pw_pass = -POW_WIND;
		order_enc_pass = ORDER_ENC_GET;

		if(-1000 < enc_pick){ //stop reverse
			m_pw_pass = 0;
			order_enc_pass = ORDER_ENC_NO_GET;
			step_launch = 0;
		}
	}

	publish_order();
}

void debug_func(){
	order_enc_pick = ORDER_ENC_NO_GET;
	order_enc_pass = ORDER_ENC_NO_GET;

	//motor
	if(debug_joy_LR == -1) m_pw_pick = debug_joy_stick*DEBUG_M_PW;
	if(debug_joy_UD ==  1) m_pw_pass = debug_joy_stick*DEBUG_M_PW;

	//solenoid
	if(debug_joy_UD == -1) sol_mode = SOLENOID_MODE_ON;
	else sol_mode = SOLENOID_MODE_OFF;
	
	//valve
	if(debug_joy_LR == 1){
		if(debug_joy_stick == 1) valve_mode = VALVE_MODE_OPEN;
		else if(debug_joy_stick == -1) valve_mode = VALVE_MODE_HOLD;
	}else{
		valve_mode = 0;
	}
}

void all_stop(){
	order_enc_pick = ORDER_ENC_NO_GET;
	order_enc_pass = ORDER_ENC_NO_GET;

	m_pw_pick  = 0;
	m_pw_pass  = 0;
	sol_mode   = 0;
	valve_mode = 0;

	step_pick   = 0;
	step_launch = 0;

	ROS_FATAL("log:%d", step_launch);
}

void publish_order(){
	data_order.data[0] = m_pw_pick;
	data_order.data[1] = m_pw_pass;
	data_order.data[2] = sol_mode;
	data_order.data[3] = valve_mode;
	data_order.data[4] = order_enc_pick;
	data_order.data[5] = order_enc_pass;

	pub_order.publish(data_order);
}
void get_joy(const sensor_msgs::Joy& msg_joy){
	order_pick    = msg_joy.buttons[1];
	order_launch  = msg_joy.buttons[0];
	emg_stop      = msg_joy.buttons[2];
	debug_mode      = msg_joy.buttons[7];
	debug_joy_LR    = msg_joy.axes[4];
    debug_joy_UD    = msg_joy.axes[5];
	debug_joy_stick = msg_joy.axes[3];
}

void get_sensor(const std_msgs::Int32MultiArray& msg_sensor){
	touch_val = msg_sensor.data[0];
	enc_pick  = msg_sensor.data[1];
	enc_pass  = msg_sensor.data[2];
}
