#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/Twist.h>
#include<tf/transform_listener.h>
#include<nav_msgs/OccupancyGrid.h>
#include<pr/RawEncoder.h>
#include<geometry_msgs/Vector3.h>
#include<math.h>
#include<vector>
#include<iostream>
//#include<> //pr_msg?
//経路追従及び障害物回避をDWAを用いて行うノード: pr_motion_planner

using namespace std;

class DWA{
public:

  nav_msgs::Path route;
  nav_msgs::OccupancyGrid LFMap;
  double accx_max, accy_max;
  double vx_max, vy_max;
  double encdt,dt; //エンコーダ周期,制御周期
  //double currvx, currvy;
  //double wheel_radius, wheel2center, resolution;
  double alpha, beta, gamma;
  int sample_rate;
  int skip; //nav_msgs::Path中のwaypointをいくつ飛ばしで使用するか
  double obstacle_thresh; //各gridの占有率(0-100)がいくつ以上だとそれを障害物とみなすか
  double kp, goal_th;
  //int isReachedGoal; //ゴールに到着したかを判定する
  int startFlag;//ナビゲーション開始のフラグ

  ros::Publisher vel_pub, end_pub;
  //pr_Msg::??? end_flag;
  double ctrvx_max, ctrvx_min, ctrvy_max, ctrvy_min;
  double vx_accmax, vx_accmin, vy_accmax, vy_accmin;
  double dx,dy;
  double expected_pos[2];
  double pos_now[3];
  int expected_pos_pix[2];
  double selected_vel[2];
  double cost, maxcost;
  double obstacle, distance, velocity;
  int curr_waypoint_no;
  geometry_msgs::Twist velcmd;
  double yaw, pitch, roll, delta_th;
  //double det, det_pre; //waypoint通過の判断に用いる変数
  double r; //ロボットがwaypointを中心とする半径rの円のなかに入ったら、次のwaypointを設定する
  //int one_time_flag;//一回だけ実行させる処理用のフラグ
  int isNavigationEnded, isCurrPosReady;
  double pos0[2], pos1[2], pos2[2], pos1_pre[2];


  void sendVelCmd(void);
  void setPath(const nav_msgs::Path::ConstPtr& path);
  //void encCb(const pr::RawEncoder::ConstPtr& encdata);
  void setCurrPos(const geometry_msgs::Vector3::ConstPtr& _pos);
  //void setOmniParams(double r, double wh2c, double resol, double edt);
  void setDWAParams(double acx_mx, double acy_mx, double maxvx, double maxvy, double t, int sr, double a, double b, double c, int skp,
    double _kp, double _r, double obs);
  void getLFMap(void);
  void initPub(ros::NodeHandle &n);

};

void DWA::setDWAParams(double acx_mx, double acy_mx, double maxvx, double maxvy, double t, int sr, double a, double b, double c, int skp,
    double _kp, double _r, double obs = 0.10){
  accx_max = acx_mx;
  accy_max = acy_mx;
  dt = t;
  sample_rate = sr;
  vx_max = maxvx; vy_max = maxvy;
  alpha = a; beta = b; gamma = c;
  skip = skp;
  obstacle_thresh = obs;
  kp = _kp;
  startFlag = 0;
  curr_waypoint_no = skip;
  //one_time_flag = 0;
  isNavigationEnded = 0;
  isCurrPosReady = 0;
  r = _r;
}

void DWA::setPath(const nav_msgs::Path::ConstPtr& path){
  cout << "motion_planner: path has been received" << endl;
  //isReachedGoal = 0;
  startFlag = 1;
  curr_waypoint_no = skip;
  //one_time_flag = 0;
  isNavigationEnded = 0;
  route = *path;
  goal_th = route.poses[route.poses.size() - 1].pose.orientation.z;
  //pos0[0] = route.poses[curr_waypoint_no - skip].pose.position.x;
  //pos0[1] = route.poses[curr_waypoint_no - skip].pose.position.y;
  pos1[0] = route.poses[curr_waypoint_no].pose.position.x;
  pos1[1] = route.poses[curr_waypoint_no].pose.position.y;
  /*pos1_pre[0] = route.poses[curr_waypoint_no - 1].pose.position.x;
  pos1_pre[1] = route.poses[curr_waypoint_no - 1].pose.position.y;
  pos2[0] = (2*pos1[0] + pos0[0])/3.0;
  pos2[1] = (2*pos1[1] + pos0[1])/3.0;*/

}

void DWA::sendVelCmd(void){

    if(startFlag == 0){//if a path hasn't been subscribed, don't publish velocity command
        //cout << "motion_palnner: the path is not received" << endl;
        return;
     }

    if(isNavigationEnded == 1){// if the goal point is passed, stops the robot
        startFlag = 0;
        velcmd.linear.x = 0.0;
        velcmd.linear.y = 0.0;
        velcmd.angular.z = 0.0;
        vel_pub.publish(velcmd);
        //end_flag.*** = 0;
        //end_pub.publish(end_flag);
        cout << "motion_planner: navigation ended" << endl;
        return;
     }
    if(isCurrPosReady == 0){
        cout << "motion_planner: current position of the robot has not been retrieved." << endl;
        return;
     }
    //Dynamic Windowの作成(ただし、障害物による条件はここでは考慮しない)
    ctrvx_max = vx_max; ctrvx_min = -vx_max;
    ctrvy_max = vy_max; ctrvy_min = -vy_max;

    /*vx_accmax = currvx + dt*accx_max;
    vx_accmin = currvx - dt*accx_max;
    vy_accmax = currvy + dt*accy_max;
    vy_accmin = currvy - dt*accy_max;

    if(vx_accmax < ctrvx_max) ctrvx_max = vx_accmax;
    if(vx_accmin > ctrvx_min) ctrvx_min = vx_accmin;
    if(vy_accmax < ctrvy_max) ctrvy_max = vy_accmax;
    if(vy_accmin > ctrvy_min) ctrvy_min = vy_accmin;*/

    //Dynamic Windowから選びうる全ての組み合わせを抽出、配列に格納
    dx = (ctrvx_max - ctrvx_min)/(double)sample_rate;
    dy = (ctrvy_max - ctrvy_min)/(double)sample_rate;
    int i = 0, j;
    vector<vector<double> > vcombi;
    vector<double> vcombi_sub(2);
    while(ctrvx_min + (double)i*dx <= ctrvx_max){
      j = 0;
      while(ctrvy_min + (double)j*dy <= ctrvy_max){
        vcombi_sub[0] = ctrvx_min + (double)i*dx;
        vcombi_sub[1] = ctrvy_min + (double)j*dy;
        vcombi.push_back(vcombi_sub);
        j++;
      }
      i++;
    }
    /*//現在位置の取得(評価関数によるコスト計算に必要)
    tf_listener.lookupTransform("map", "base_link", ros::Time(0), transf);
    pos_now[0] = transf.getOrigin().x();
    pos_now[1] = transf.getOrigin().y();
    tf::Matrix3x3 m(transf.getRotation());
    m.getRPY(roll,pitch,yaw);
    pos_now[2] = yaw;*/


    selected_vel[0] = 0.0; selected_vel[1] = 0.0;//障害物に囲まれて動けない場合、並進移動をしない
    maxcost = 0.0;
    //各制御コマンドの組み合わせに対し、予測&評価を行う
    for(int k = 0; k < vcombi.size(); k ++){
      //各制御コマンドの組み合わせに対し、動作予測を行う
      expected_pos[0] = pos_now[0] + dt*vcombi[k][0];
      expected_pos[1] = pos_now[1] + dt*vcombi[k][1];
       //convert to grid map position
      expected_pos_pix[0] = (int)((fabs(LFMap.info.origin.position.x) + expected_pos[0])/LFMap.info.resolution) - 1;
      expected_pos_pix[1] = (int)((fabs(LFMap.info.origin.position.y) + expected_pos[1])/LFMap.info.resolution) - 1;


      //評価関数による予測評価(コストの算出)
      obstacle = 1.0 - ((double)(LFMap.data[expected_pos_pix[1]*LFMap.info.width + expected_pos_pix[0]])/100.0);
      velocity = (vcombi[k][0]*vcombi[k][0] + vcombi[k][1]*vcombi[k][1])/(vx_max*vx_max + vy_max*vy_max);
      distance = exp(-(route.poses[curr_waypoint_no].pose.position.x - expected_pos[0])*(route.poses[curr_waypoint_no].pose.position.x - expected_pos[0])
       - (route.poses[curr_waypoint_no].pose.position.y - expected_pos[1])*(route.poses[curr_waypoint_no].pose.position.y - expected_pos[1]));

      cost = alpha*obstacle + beta*velocity + gamma*distance;
      //cout << "motion_planner: vx=" << vcombi[k][0] << "vy=" << vcombi[k][1] << ":obs=" << obstacle << endl;
      if(obstacle < obstacle_thresh) cost = 0.0; //各gridの占有率が100*(1-obstacle_thresh)%以上の時、costの値を0とする
      if(cost > maxcost){//最もコストが高かった制御コマンドの組み合わせを採用する
        maxcost = cost;
        selected_vel[0] = vcombi[k][0];
        selected_vel[1] = vcombi[k][1];
       }
     }
    velcmd.linear.x = selected_vel[0];
    velcmd.linear.y = selected_vel[1];
    delta_th = pos_now[2] - goal_th;
    if(delta_th > 3.1415) delta_th = -(2*3.1415 - pos_now[2] + goal_th);
    if(delta_th < -3.1415) delta_th = 2*3.1415 + pos_now[2] - goal_th;
    velcmd.angular.z = -kp*delta_th;

    vel_pub.publish(velcmd);


}

/*void DWA::encCb(const pr::RawEncoder::ConstPtr& enc){
  //4輪オムニの運動モデルを用いる
  double vel1 = 2.0*3.1415*wheel_radius*enc->e1/(resolution*encdt);
  double vel2 = 2.0*3.1415*wheel_radius*enc->e2/(resolution*encdt);
  double vel3 = 2.0*3.1415*wheel_radius*enc->e3/(resolution*encdt);

  currvx = (vel3 - vel2)/sqrt(2.0);
  currvy = (vel1 - vel2)/sqrt(2.0);
}

void DWA::setOmniParams(double r, double wh2c, double resol, double edt){
  wheel_radius = r;
  wheel2center = wh2c;
  resolution = resol;
  encdt = edt;
}*/

void DWA::getLFMap(void){
  boost::shared_ptr<nav_msgs::OccupancyGrid const> mapptr;
  cout << "motion_planner: getting map..." << endl;
  mapptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("LFMap");
  LFMap = *mapptr;
}

void DWA::setCurrPos(const geometry_msgs::Vector3::ConstPtr& _pos){
  isCurrPosReady = 1;
  pos_now[0] = _pos->x;
  pos_now[1] = _pos->y;
  pos_now[2] = _pos->z;

  //ロボットがwaypointを通過したと判断された場合、次のwaypointを設定する
  if(startFlag){
    if(( (pos_now[0] - pos1[0])*(pos_now[0] - pos1[0]) + (pos_now[1] - pos1[1])*(pos_now[1] - pos1[1]) ) < r*r ){

      if(curr_waypoint_no == route.poses.size() - 1){//ゴールを通過したらロボットを停止させる
        isNavigationEnded = 1;
        return;
      }
      
      cout << "motion_planner: passed waypoint no. " << curr_waypoint_no << endl;
      curr_waypoint_no += skip;
      if(curr_waypoint_no > route.poses.size() - 1) curr_waypoint_no = route.poses.size() - 1;

      pos1[0] = route.poses[curr_waypoint_no].pose.position.x;
      pos1[1] = route.poses[curr_waypoint_no].pose.position.y;
    }
  }
  /*if(pos0[1] != pos1[1]){
    det = pos_now[1] - pos2[1] + (pos1[0] - pos1_pre[0])*(pos_now[0] - pos2[0])/(pos1[1] - pos1_pre[1]);

  }else if(pos0[1] == pos1[1]){
    det = pos_now[0] - pos2[0];
  }
  if(one_time_flag == 0){
    det_pre = det;
    one_time_flag++;
  }
  if(det*det_pre < 0){//waypointを通過したか？
    //cout << "motion_planner: passed waypoint no. " << curr_waypoint_no << endl;
    if(curr_waypoint_no == route.poses.size() - 1){//ゴールを通過したらロボットを停止させる
      isNavigationEnded = 1;
      det_pre = det;
      return;
    }
    curr_waypoint_no += skip;// update curr_waypoint_no
    one_time_flag = 0;
    if(curr_waypoint_no > route.poses.size() - 1) curr_waypoint_no = route.poses.size() - 1;

    pos0[0] = route.poses[curr_waypoint_no - skip].pose.position.x;
    pos0[1] = route.poses[curr_waypoint_no - skip].pose.position.y;
    pos1[0] = route.poses[curr_waypoint_no].pose.position.x;
    pos1[1] = route.poses[curr_waypoint_no].pose.position.y;
    pos1_pre[0] = route.poses[curr_waypoint_no - 1].pose.position.x;
    pos1_pre[1] = route.poses[curr_waypoint_no - 1].pose.position.y;
    pos2[0] = (2.0*pos1[0] + pos0[0])/3.0;
    pos2[1] = (2.0*pos1[1] + pos0[1])/3.0;

  }
  det_pre = det;*/

}

void DWA::initPub(ros::NodeHandle &n){
  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  //end_pub = n.advertise<***>("", 10);
}


DWA dwa;
void _setPath(const nav_msgs::Path::ConstPtr& path){
  dwa.setPath(path);
}
/*void _encCb(const pr::RawEncoder::ConstPtr& enc){
  dwa.encCb(enc);
}*/

void _posCb(const geometry_msgs::Vector3::ConstPtr& _pos){
  dwa.setCurrPos(_pos);
}

void timerCb(const ros::TimerEvent&){
  dwa.sendVelCmd();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pr_motion_planner");
  ros::NodeHandle nh;

  double maxvx, maxvy, ctrl_t, alpha, beta, gamma, kp, r, obs_thr;
  int sample_rate, skip;
  double w2c, wr, enc_resol;

  nh.getParam("/const/dwa/max_vel_x", maxvx);
  nh.getParam("/const/dwa/max_vel_y", maxvy);
  nh.getParam("/const/dwa/ctrl_period", ctrl_t);
  nh.getParam("/const/dwa/sample_rate", sample_rate);
  nh.getParam("/const/dwa/coeff_obs", alpha);
  nh.getParam("/const/dwa/coeff_vel", beta);
  nh.getParam("/const/dwa/coeff_dist", gamma);
  nh.getParam("/const/dwa/skip_wp", skip);
  nh.getParam("/const/dwa/kp", kp);
  nh.getParam("/const/spec/center_to_wheel", w2c);
  nh.getParam("/const/spec/wheel_radius", wr);
  nh.getParam("/const/spec/encoder_resolution", enc_resol);
  nh.getParam("/const/dwa/waypoint_range", r);
  nh.getParam("/const/dwa/obstacle_thresh", obs_thr);
  dwa.setDWAParams(1.0, 1.0, maxvx, maxvy, ctrl_t, sample_rate, alpha, beta, gamma, skip, kp, r, obs_thr);//左から順に、x,y軸方向の最大加速度、x,y軸方向の最大速度、制御周期(秒)、サンプリングレート、評価関数の各要素の重み、パスを構成する点をいくつ飛ばしで用いるか,回転角度制御(P制御)におけるPゲイン
  //dwa.setOmniParams(wr, w2c, enc_resol, 0.005);
  dwa.initPub(nh);

  ros::Subscriber path_sub = nh.subscribe("path", 10, _setPath);
  //ros::Subscriber enc_sub = nh.subscribe("raw_encoder", 1000, _encCb);
  ros::Subscriber pos_sub = nh.subscribe("current_pos", 1000, _posCb);
  ros::Timer timer = nh.createTimer(ros::Duration(ctrl_t), timerCb);

  dwa.getLFMap();

  ros::spin();
}
