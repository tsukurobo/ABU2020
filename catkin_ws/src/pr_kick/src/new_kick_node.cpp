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
const int SOLENOID_LOCK_OFF = 0;
const int SOLENOID_LOCK_ON = 1;
const int ENC_PER_ROT = 4048;
const int DEBUG_M_PW = 100;
const int ORDER_ENC_GET    = 1;
const int ORDER_ENC_NO_GET = 0;

//sub joy
//task order
int order_wind   = 0;
int order_launch = 0;
int emg_stop     = 0;
//debug order
int debug_mode = 0;
int debug_joy_LR;
int debug_joy_UD;
float debug_joy_stick;
int debug_enc = 0;

//pub arduino
//actuator
int m_pw;
int sol_mode;
//encoder order
int order_enc;

//sub sensor
int touch_val;
long enc_val;

//step
int step_wind    = 0;
int step_launch  = 0;

//parameter
int POW; //巻取るモータパワー (-255~255) 
int DEG; //緩め解除する角度[deg]
int DELAY_SOL_1; //ソレノイドonの時間[milli sec]
int DELAY_SOL_2; //ソレノイドoffの時間[milli sec]

//function protype
void get_joy(const sensor_msgs::Joy& msg_joy);
void get_sensor(const std_msgs::Int32MultiArray& msg_sensor);
void task_wind();
void task_launch();
void debug_func();
void all_stop();
void publish_order();

int main(int argc, char **argv){
	//node init
	ros::init(argc,argv,"new_kick_node");
	ros::NodeHandle nh;

	sub_joy    = nh.subscribe("joy",1,get_joy);
	sub_sensor = nh.subscribe("kick_sensor",1,get_sensor);
	pub_order  = nh.advertise <std_msgs::Int32MultiArray>("kick_order",1);

	//paramerter
	nh.getParam("new_kick_node/POW",POW);
	nh.getParam("new_kick_node/DEG",DEG);
	nh.getParam("new_kick_node/DELAY_SOL_1",DELAY_SOL_1);
	nh.getParam("new_kick_node/DELAY_SOL_2",DELAY_SOL_2);

	ros::Rate loop_rate(FREQ);

	data_order.data.resize(3);
	data_order.data[0] = 0;
	data_order.data[1] = 0;
	data_order.data[2] = 0;

	//node body
	while(ros::ok()){
		ros::spinOnce();
	order_enc = ORDER_ENC_GET;

		//mode change
		if(emg_stop == 1){ //emg_stop mode
			all_stop();
			publish_order();
		}else if(debug_mode == 1){ //debug mode
			debug_func();
			publish_order();
		}else{
			if(order_wind   == 1 || step_wind > 0)   task_wind();
			if(order_launch == 1 || step_launch > 0) task_launch();
		}



		loop_rate.sleep();
	}

	return 0;
}

void task_wind(){
	if(step_wind == 0){
		step_wind = 1;
	}else if(step_wind == 1){ //normal rotation
		m_pw = POW;

		if(touch_val == SOLENOID_LOCK_ON){
			m_pw = 0;
			step_wind = 2;
		}
	}else if(step_wind == 2){ //reverse rotation
		m_pw = -POW;        //lower hand
		//order_enc = ORDER_ENC_GET;

		if(DEG*ENC_PER_ROT/360.0 < enc_val){ //stop hand
			m_pw = 0;
			//order_enc = ORDER_ENC_NO_GET;
			step_wind = 0;
		}
	}

	//ROS_FATAL("log:%d",step_wind);

	publish_order();

}

void task_launch(){
	if(step_launch == 0){
		step_launch = 1;
	}else if(step_launch = 1){
		static int cnt = 0;
		cnt++;

		if(touch_val == SOLENOID_LOCK_ON){ //launch ball
			if(DELAY_SOL_1*FREQ/1000 > cnt) sol_mode = SOLENOID_MODE_ON;
			else if(DELAY_SOL_2*FREQ/1000 > cnt) sol_mode = SOLENOID_MODE_OFF;
			else cnt = 0;
			//ROS_FATAL("sol_mode:%d",sol_mode);
		}else{
			cnt = 0;
			step_launch = 0;
			//ROS_FATAL("bbb");
		}
	}

	//ROS_FATAL("log:%d",step_launch);

	publish_order();
	sol_mode = 0;
}

void debug_func(){
	//motor
	if(debug_joy_LR == -1) m_pw = debug_joy_stick*DEBUG_M_PW;

	//solenoid
	if(debug_joy_UD == 1) sol_mode = SOLENOID_MODE_ON;
	else sol_mode = SOLENOID_MODE_OFF;
	
	//encoder
	if(debug_enc == 1){
		order_enc = ORDER_ENC_GET;
	}else{
		order_enc = ORDER_ENC_NO_GET;
	}
}

void all_stop(){
	order_enc = ORDER_ENC_NO_GET;

	m_pw  = 0;
	sol_mode   = 0;

	step_wind   = 0;
	step_launch = 0;
}

void publish_order(){
	data_order.data[0] = m_pw;
	data_order.data[1] = sol_mode;
	data_order.data[2] = order_enc;

	pub_order.publish(data_order);
}
void get_joy(const sensor_msgs::Joy& msg_joy){
	order_wind    = msg_joy.buttons[5];
	order_launch  = msg_joy.buttons[4];
	emg_stop      = msg_joy.buttons[2];
	debug_mode      = msg_joy.buttons[6];
	debug_joy_LR    = msg_joy.axes[4];
    debug_joy_UD    = msg_joy.axes[5];
	debug_joy_stick = msg_joy.axes[3];
	debug_enc = msg_joy.buttons[3];
}

void get_sensor(const std_msgs::Int32MultiArray& msg_sensor){
	touch_val = msg_sensor.data[0];
	enc_val   = msg_sensor.data[1];
}
