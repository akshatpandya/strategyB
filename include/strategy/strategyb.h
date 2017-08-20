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

  void retrieve_pose(nav_msgs::Odometry *gbpose);
  void groundbotCallback(const nav_msgs::Odometry::ConstPtr& msg);

	void nav_quad_callback(const strategy::navigate_quad::ConstPtr& msg);
  void feedbackfn(const nav_msgs::Odometry::ConstPtr& odom_data);

  ros::NodeHandle n;
  ros::Subscriber imu_yaw;
  ros:: Subscriber sub;
  ros:: Publisher publi;
  ros:: Publisher botToTap;
	ros:: Subscriber nav_quad;

  void initialHerd();
  int findBotNearestToQuad(int hover);
  void whereToTurn(int ID);
	void sendQuad(int id, int mode, char reached, double x, double y, double z);
	void GetEulerAngles(qt q, double* yaw, double* pitch, double* roll);
	float angle(float ang);
	void greedy();
	int translateFrame();
	int removeTheLockedBot(int ID);
	int isOutsideGreen(int ID);

  strategyb()
  {
    sub = n.subscribe("robot/odom", 100, &strategyb::groundbotCallback,this);
    imu_yaw = n.subscribe("mavros/local_position/odom", 10, &strategyb::feedbackfn,this);
		nav_quad = n.subscribe("groundbot/tap", 100, &strategyb::nav_quad_callback,this);
    botToTap = n.advertise<strategy::navigate_quad>("groundbot/tap",1000, true);
    firstrun = true;
		flag = 0;
		lockID = 0;
		right = true;
		left = false;
		up = false;
		thresx = 2.5;
		thresy = 1;
	  frameX = new double;
	 	frameY = new double;
		*frameX = 0;
		*frameY = 0;
		sequence = 0;
  }


private:
  nav_msgs::Odometry bot_pose;
  nav_msgs::Odometry MAVpose;
	strategy::navigate_quad mvpose;

	typedef pair <double, int> p;
	set<p> ClosestBot;
  int flag, lockID, thresx, thresy;
  bool firstrun;
	double *frameX, *frameY;
	bool right, left, up;
	unsigned int sequence;
};
#endif
