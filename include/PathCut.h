#ifndef _PATH_CUT_H
#define _PATH_CUT_H

#include <vector>
#include <queue>
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <cmath>

// ros header
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>

using std::vector;
using std::pair;
using std::cout;
using std::endl;
using std::unordered_set;
using std::ifstream;

const double eps = 1e-7;

typedef vector<vector<int>> Mat;
typedef pair<int, int> P;
typedef struct Cut {
	int start;		//ע�⣬�������Բ�ϵı��
	int len;
	double val;		//�Ƚϵ�ʱ����eps��ֹ�������
} cut ;

typedef struct Node {
	double fx;
	double gx;
	int id;
	bool operator <(const Node& n)const {
		return fx - n.fx > eps;
		//return fx < n.fx;
	}
} node ;

#define reshape(i, j) (int)((i) * bigcols + (j))
#define PI 3.1415926

static double ONE_TURN_VAL = 2.0;

class PathCut {
	int bigrows, bigcols;
	int smallrows, smallcols;
	int circleLen;
	Mat MST, Map, Region;
	Mat pathEdge;
	vector<int> depot;
	vector<int> pathSequence, invSequence;
	vector<cut> cuts;
	vector<double> pathValue;
	//int dir[8][2] = { 0, 1, 0, -1, 1, 0, -1, 0, 1, 1, 1, -1, -1, 1, -1, -1};
	int dir[4][2] = { {0, 1}, {0, -1}, {1, 0}, {-1, 0} };

	vector<int> depot_cut;  // from depot to cuts
	vector<int> cut_depot;

	bool useROSPlanner;
	bool coverAndReturn;
	ros::NodeHandle* nh;
	nav_msgs::OccupancyGridPtr globalCostmap;

public:
	PathCut(Mat& map, Mat& region, Mat& tree, vector<int>& robotInitPos, ros::NodeHandle* _nh, nav_msgs::OccupancyGridPtr _globalCostmap, bool _useROSPlanner = false, bool _coverAndReturn = false) : 
				Map(map), Region(region), MST(tree), depot(robotInitPos), nh(_nh), globalCostmap(_globalCostmap), useROSPlanner(_useROSPlanner), coverAndReturn(_coverAndReturn) {
		bigrows = Region.size();
		bigcols = Region[0].size();
		smallrows = Map.size();
		smallcols = Map[0].size();
		circleLen = 0;
		cuts.resize(depot.size(), {}); 
		depot_cut.resize(depot.size(), 0);	// ��i�������˶�Ӧ��Բ�ϵı��
		cut_depot.resize(depot.size(), 0);	// Բ�ϵ�i��cut��Ӧ�Ļ�����ԭ���
	}

	void MST2Path();
	void get2DCoordinate(int index, int& x, int& y);
	void MSTC_Star();
	void Balanced_Cut(vector<int>& adjustCuts);
	double updateCutVal(int i);
	double A_star(int u, int v);
	vector<int> A_star_path(int u, int v);
	vector<int> getHalfCuts(int cut_min, int cut_max, int dir);
	Mat generatePath();
	double euclidean_dis(double x1, double y1, double x2, double y2);
	Mat cutSolver();
	int getTurnsNum();
	double getTurnAndLength(int i);
	double ROSGlobalPlannerCost(int start_id, int goal_id);
	double estimatePathCost(nav_msgs::Path& path);

	bool isSameLine(int a, int b, int c) {
		return a + c == 2 * b;
	}
};

#endif