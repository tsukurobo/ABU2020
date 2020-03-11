#include <iostream>
#include <queue>
#include <vector>
#include <functional>
#include <fstream>
#include <string>
#include <unordered_map>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <sstream>
#include <tf/transform_listener.h>

using namespace std;

//A*により最短経路を計算するノード:pr_global_planner

vector<string> split(string s, char dev){
	int first = 0;
	int last = s.find_first_of(dev);

	vector<string> result;

	while (first < s.size()){
		string subStr(s, first, last - first);

		result.push_back(subStr);

		first = last + 1;
		last = s.find_first_of(dev, first);

		if (last == string::npos){
			last = s.size();
		}
	}

	return result;
}


class A_star{ //A*アルゴリズムを手軽に扱うためのクラス(ROS仕様)
public:
    
    typedef struct Result_Path{
	vector<string> path;
	double min_cost;
    }Result_Path;

    class Node{
    public:
        unordered_map<string, double> edges;
        std::vector<int> pos;//クラス内でのvectorの初期化は出来ない
		double cost;
		string from;
		double move_cost;
		bool operator > (const Node &obj) const{ //ノード同士の比較を行うための演算子定義
	    	return this->cost > obj.cost;
		}
		bool operator < (const Node &obj) const{
	    	return this->cost < obj.cost;
		}

        Node(){
	    	pos.resize(2);
	    	cost = -1;
	    	move_cost = 0;
        }
    };

    nav_msgs::OccupancyGrid map;
    const double cost_ty = 1.0, cost_n = 1.42; //cost_ty:縦横方向の移動コスト。cost_n:斜め方向の移動コスト。
    double m_per_pixel = 0.0; //１ピクセルあたりの長さ[m]
    int map_w = 0, map_h = 0; //マップの縦・横のピクセル数
    double origin_x = 0.0, origin_y = 0.0; //map座標系の原点から見た、マップの左端の座標
    int nodeNum = 0;
	int obs_thresh = 90;
    
    double heuristic(vector<int> p, vector<int> goalp, bool mode){ //ヒューリスティック関数。移動コストの付け方によって変える。
		if (mode == true){ //if true, uses A* algolithm
	    	return sqrt((double)((p[0]-goalp[0])*(p[0]-goalp[0])+(p[1]-goalp[1])*(p[1]-goalp[1])));
	    	//return (double)(abs(p[0] - goalp[0]));
		}else{ //if false, uses dijkstra's algorithm
	    	return 0.0;
		}
    }

    void examine_surrounding_pixels(Node& n, unordered_map<string, Node>& m){ //引数のノードの周囲にある8つのピクセルを調べる
		ostringstream oss;
		double dp[8][3] = {{1,0,cost_ty},{1,1,cost_n},{0,1,cost_ty},{-1,1,cost_n},{-1,0,cost_ty},{-1,-1,cost_n},{0,-1,cost_ty},{1,-1,cost_n}}; //座標の増分、移動コストの順
		int pix_x = 0, pix_y = 0;
		string nodeName;
	
		for(int i=0; i < 8; i++){
	  		pix_x = n.pos[0] + (int)dp[i][0];
	  		pix_y = n.pos[1] + (int)dp[i][1];
	  
			if (map.data[pix_y*map_w + pix_x] == 0){
	        	oss.str("");
	        	oss.clear(stringstream::goodbit);
				oss << pix_x << "," << pix_y;
				nodeName = oss.str();
		
				if(m.find(nodeName) == m.end()){ //連想配列にノードがない場合、新しく生成する
		    		Node newNode;

		    		newNode.cost = -1;
		    		newNode.pos[0] = pix_x; newNode.pos[1] = pix_y;
		
		    		m[nodeName] = newNode;
				}
				n.edges[nodeName] = dp[i][2];//元のノードのedges要素にノード情報を追加
	    	}
		}
	
    }

    
    nav_msgs::Path solve(vector<double> start, vector<double> end, bool mode){ //スタートノードとエンドノード間の最適経路を出力
	string name;
	double cost;
	double move_cost;
	string to;
	vector<int> gpp(2,0); //ゴールノードのピクセル座標を格納。goal_position_pixel
	priority_queue<Node, vector<Node>, greater<Node> > q;
	Node doneNode;
	ostringstream oss;
	unordered_map<string, Node> mp;
	nav_msgs::Path path;

	gpp[0] = (int)((fabs(origin_x) + end[0])/m_per_pixel) - 1;
	gpp[1] = (int)((fabs(origin_y) + end[1])/m_per_pixel) - 1;
	
	//スタートノードの作成
	Node start_n;
    start_n.pos[0] = (int)((fabs(origin_x) + start[0])/m_per_pixel) - 1;//スタートノードの座標をピクセル座標に変換
	start_n.pos[1] = (int)((fabs(origin_y) + start[1])/m_per_pixel) - 1;
	if(map.data[gpp[1]*map_w + gpp[0]] >=  70 || map.data[start_n.pos[1]*map_w + start_n.pos[0]] >= 70 ){ //ゴールやスタート地点にロボットが存在できない場合、パスは計算しない
		cout << "global_planner: invalid goal or start point" << endl;
		path.header.seq = 100;
		return path;
	}
	start_n.cost = heuristic(start_n.pos, gpp, mode);
	start_n.move_cost = 0.0;
	start_n.from = "null";

	oss << start_n.pos[0] << "," << start_n.pos[1]; //連想配列用のキーの作成
    mp[oss.str()] = start_n;//連想配列と優先度付きキューにスタートノードを追加
	q.push(start_n);
	//cout << "global_planner: created a start node" << endl;

	//アルゴリズムの本体
	while (1){
	    doneNode = q.top();//最小コストを持つノードを確定ノードとする
	    q.pop();
	    if (doneNode.pos == gpp) break;//エンドノードが確定ノードになったら終わり

	    oss.str("");
	    oss.clear(stringstream::goodbit);
	    oss << doneNode.pos[0] << "," << doneNode.pos[1];
	    name = oss.str();
	    
	    examine_surrounding_pixels(mp[name], mp);//ノードの周辺にあるピクセルを調べる.と同時に、適切なノードの生成も行う
	    
	    for (auto itr = mp[name].edges.begin(); itr != mp[name].edges.end(); ++itr){//確定ノードに接続されている全ノードに対し、次を実行
		move_cost = itr->second + mp[name].move_cost;
		to = itr->first;
		cost = heuristic(mp[to].pos, gpp, mode) + move_cost;

		if (cost < mp[to].cost || mp[to].cost < 0){//ノードのコストの更新
		    mp[to].cost = cost;
		    mp[to].move_cost = move_cost;
		    q.push(mp[to]);
		    mp[to].from = name;
		}
	    }
	}
	//cout << "global_planner: finished calculation" << endl;
	//パスと最小コストの出力
	Result_Path tmp;
	oss.str("");
	oss.clear(stringstream::goodbit);
	oss << gpp[0] << "," << gpp[1];
	Node n = mp[oss.str()];
	tmp.path.push_back(oss.str());
	int tmpSize = 0;
	vector<string> name_divided;
	geometry_msgs::PoseStamped ps;
	//double min_cost = 0;

	while (n.from != "null"){
	    tmp.path.push_back(n.from);
	    n = mp[n.from];
	}
	
	tmpSize = tmp.path.size();
	
	for (int i = 0; i < tmpSize; i++){
	    
	    
	    name_divided = split(tmp.path[tmpSize - 1 - i],',');
	    
	    ps.pose.position.x = m_per_pixel*atoi(name_divided[0].c_str()) - fabs(origin_x);
	    ps.pose.position.y = m_per_pixel*atoi(name_divided[1].c_str()) - fabs(origin_y); //split()で返されるのは、,を抜いたstringのvector
	    //cout << name << endl;
	    ps.pose.position.z = 0.0;
	    
	    path.poses.push_back(ps);
	}
	
	//path.min_cost = nodes[end].cost;
	path.header.seq = 0;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "map";
	path.poses[path.poses.size() - 1].pose.orientation.z = end[2];
	
	nodeNum = mp.size();
	//cout << "global_planner: created a path message" << endl;
	return path;
	
    }

    int readGridMap(void){
	boost::shared_ptr<nav_msgs::OccupancyGrid const> pathptr;

	pathptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("LFMap"); //テンプレートも忘れずに

	if(pathptr != NULL){
	    map = *pathptr;
	    m_per_pixel = map.info.resolution;
	    map_w = map.info.width;
	    map_h = map.info.height;
	    origin_x = map.info.origin.position.x;
	    origin_y = map.info.origin.position.y;
	    return 1;
	}
	else{
	    return 0;
	}
    }

};







A_star a_star;
ros::Publisher path_pub;
int flag = 0; //whether publish a path or not
nav_msgs::Path path;
int isMapLoaded = 0;
vector<double> currpos(2);

void goalcb(const geometry_msgs::Vector3::ConstPtr& goalpos){
    vector<double> gpos(3);
    vector<double> spos(2,0.0); //2つの要素全てに0.0を代入
    double time_s, time_g;

    gpos[0] = goalpos->x;
    gpos[1] = goalpos->y;
    gpos[2] = goalpos->z;
        
    spos = currpos;

    if(!isMapLoaded){
		cout << "global_planner: loaded map" << endl;
		isMapLoaded = a_star.readGridMap();
     }

    if(isMapLoaded){
	 	flag = 1;
	 	time_s = ros::Time::now().toSec();
	 	path = a_star.solve(spos,gpos,true);
	 	if(path.header.seq == 0){
	 		time_g = ros::Time::now().toSec();
	 		cout << "global_planner: time required:" << time_g - time_s << endl;
	 		cout << "global_planner: number of nodes searched:" << a_star.nodeNum << endl;
	 		path_pub.publish(path);
		 }
    }else{
	 ROS_WARN("Couldn't get a map.");
     }
}

void posCb(const geometry_msgs::Vector3::ConstPtr& cpos){
    currpos[0] = cpos->x;
    currpos[1] = cpos->y;
}



int main(int argc, char **argv){
    
    ros::init(argc, argv,"pr_global_planner");
    ros::NodeHandle nh;
    ros::Subscriber goal_sub, pos_sub;
    ros::Rate loop_rate(10);
    //tf::TransformListener tf_listener; //You have to define TransformListener outside callback function. Otherwise, you'll get a error.
    //tf::StampedTransform trans;
    
    path_pub = nh.advertise<nav_msgs::Path>("path",10);
    goal_sub = nh.subscribe("goal",10,goalcb);
    pos_sub = nh.subscribe("current_pos", 1000, posCb);
    
    /*while (ros::ok()){
       try{
           tf_listener.lookupTransform("map", "base_link", ros::Time(0), trans);
           currpos[0] = trans.getOrigin().x();
           currpos[1] = trans.getOrigin().y();
       }catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
 
	if(flag){
	    path_pub.publish(path);
	}

	ros::spinOnce();
	loop_rate.sleep();
    }*/
    ros::spin();
	
    return 0;
}
