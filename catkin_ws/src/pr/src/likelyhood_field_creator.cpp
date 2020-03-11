#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<math.h>
#include<iostream>

//尤度場付きグリッドマップを生成するノード:likelyhood_field_creator

void convertToLFMap(nav_msgs::OccupancyGrid *_map, int n){
  int w = _map->info.width, h = _map->info.height;
  int pos_c[2] = {0, 0}; //position_centerの略
  int p = 0;

  for(int i = 0; i < w*h; i++){

    if(_map->data[i] == 100){
      pos_c[0] = i % w; pos_c[1] = i/w;

      for(int j = pos_c[1] - n; j <= pos_c[1] + n; j++){
        for(int k = pos_c[0] - n; k <= pos_c[0] + n; k++){
          p = (int)100*exp(-0.5*((pos_c[0] - k)*(pos_c[0] - k) + (pos_c[1] - j)*(pos_c[1] - j))/(double)(n*n));
          if (p > _map->data[j*w + k]) _map->data[j*w + k] = p;
        }
      }
    }
  }

}


int main(int argc, char **argv){
  ros::init(argc, argv, "pr_likelyhood_field_creator");
  ros::NodeHandle nh;
  ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("LFMap", 10);
  nav_msgs::OccupancyGrid map;
  boost::shared_ptr<nav_msgs::OccupancyGrid const> mapptr;
  ros::Rate rate(5);
  double time_pre = 0.0;
  int range;

  nh.getParam("/const/LF_creator/range", range);
  mapptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map");
  map = *mapptr;
  time_pre = ros::Time::now().toSec();
  convertToLFMap(&map, range);//二つ目のパラメータで、尤度を設定する範囲を決める
  std::cout << "LF_creator: time required: " << ros::Time::now().toSec() - time_pre << std::endl;

  while(ros::ok()){
    map_pub.publish(map);
    rate.sleep();
  }

}
