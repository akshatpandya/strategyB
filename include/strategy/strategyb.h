#ifndef STRATEGYB_H
#define STRATEGYB_H

#include <iostream>
#include <time.h>
#include <math.h>
#include <utility>
#include <set>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "strategy/navigate_quad.h"
//#include "../include/strategy/flyquad.h" //!!!!!!!!!change the location

using namespace std;

typedef struct
{
	double x,y,z,w;
}qt;

class strategyb{

public:

  void retrieve_pose(int ID, nav_msgs::Odometry *gbpose);
  void groundbot4Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void groundbot5Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void groundbot6Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void groundbot7Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void groundbot8Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void groundbot9Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void groundbot10Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void groundbot11Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void groundbot12Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void groundbot13Callback(const nav_msgs::Odometry::ConstPtr& msg);
	void nav_quad_callback(const strategy::navigate_quad::ConstPtr& msg);
  void feedbackfn(const nav_msgs::Odometry::ConstPtr& odom_data);

  ros::NodeHandle n;
  ros::Subscriber imu_yaw;
  ros:: Subscriber sub_4;
  ros:: Subscriber sub_5;
  ros:: Subscriber sub_6;
  ros:: Subscriber sub_7;
  ros:: Subscriber sub_8;
  ros:: Subscriber sub_9;
  ros:: Subscriber sub_10;
  ros:: Subscriber sub_11;
  ros:: Subscriber sub_12;
  ros:: Subscriber sub_13;
  ros:: Publisher publi;
  ros:: Publisher botToTap;
	ros:: Subscriber nav_quad;

  void initialHerd();
  int findBotNearestToQuad(int hover);
  void whereToTurn(int ID);
	void sendQuad(int id, int mode, char reached, double x, double y, double z);
	void GetEulerAngles(qt q, double* yaw, double* pitch, double* roll);
  void rotate (double relative_angle, char publish_name[40], int ID);
	float angle(float ang);
	void greedy();
	void translateFrame();
	int removeTheLockedBot(int ID);
	int isOutsideGreen(int ID);

  strategyb()
  {
    sub_4 = n.subscribe("robot4/odom", 100, &strategyb::groundbot4Callback,this);
    sub_5 = n.subscribe("robot5/odom", 100, &strategyb::groundbot5Callback,this);
    sub_6 = n.subscribe("robot6/odom", 100, &strategyb::groundbot6Callback,this);
    sub_7 = n.subscribe("robot7/odom", 100, &strategyb::groundbot7Callback,this);
    sub_8 = n.subscribe("robot8/odom", 100, &strategyb::groundbot8Callback,this);
    sub_9 = n.subscribe("robot9/odom", 100, &strategyb::groundbot9Callback,this);
    sub_10 = n.subscribe("robot10/odom", 100, &strategyb::groundbot10Callback,this);
    sub_11 = n.subscribe("robot11/odom", 100, &strategyb::groundbot11Callback,this);
    sub_12 = n.subscribe("robot12/odom", 100, &strategyb::groundbot12Callback,this);
    sub_13 = n.subscribe("robot13/odom", 100, &strategyb::groundbot13Callback,this);
    imu_yaw = n.subscribe("mavros/local_position/odom", 10, &strategyb::feedbackfn,this);
		nav_quad = n.subscribe("groundbot/tap", 100, &strategyb::nav_quad_callback,this);
    botToTap = n.advertise<strategy::navigate_quad>("groundbot/tap",1000, true);
    firstrun = true;
		flag = 0;
		lockID = 0;
		right = true;
		left = false;
		up = false;
		thresx = 5;
		thresy = 1;
	  	frameX = new double;
	  	frameY = new double;
		*frameX = 0;
		*frameY = 0;
  }


private:
  nav_msgs::Odometry gb4pose;
  nav_msgs::Odometry gb5pose;
  nav_msgs::Odometry gb6pose;
  nav_msgs::Odometry gb7pose;
  nav_msgs::Odometry gb8pose;
  nav_msgs::Odometry gb9pose;
  nav_msgs::Odometry gb10pose;
  nav_msgs::Odometry gb11pose;
  nav_msgs::Odometry gb12pose;
  nav_msgs::Odometry gb13pose;
  nav_msgs::Odometry MAVpose;
	strategy::navigate_quad mvpose;
	typedef pair <double, int> p;
	set<p> ClosestBot;
  int flag, lockID, thresx, thresy;
  bool firstrun;
	double *frameX, *frameY;
	bool right, left, up;
};
#endif
