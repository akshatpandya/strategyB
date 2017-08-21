#include <iostream>
#include <math.h>
#include <utility>
#include <set>
#include <math.h>
#include <chrono>
#include <thread>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "../include/strategy/strategyb.h"

#define PI 3.14159
#define angular_speed 0.76
using namespace std;
using namespace std::this_thread;
using namespace std::chrono;

//put a check for if nothing found in the first greedy run

int strategyb::findBotNearestToQuad(int hover)
{
  nav_msgs::Odometry position;
  ros::Rate loop_rate(10);
  do
  {
    time_t t1, t2, timer;
    while(ros::ok() && timer<=10)
    {
      time(&t1);
      ros::spinOnce();
      retrieve_pose(&position);
      if(position.header.seq!=sequence)
      {
        sequence = position.header.seq;
        return 4;
      }
      timer = time(&t2) - t1;
    }
  }while(ros::ok() && hover==1);
  return 0;
}

void strategyb::initialHerd()
{
  sendQuad(1, 0, 'n', 0, -3, 2);
  ROS_INFO("Going to 0,-3 m\n");
  ros::Rate loop_rate(10);
  ros::spinOnce();

  while(ros::ok() && !(mvpose.reached=='y'))
  	ros::spinOnce();

  int ID = findBotNearestToQuad(1);

  ROS_INFO("Found nearest bot %d\n", ID);

  loop_rate.sleep();
  whereToTurn(ID);
}

void strategyb::greedy()
{
  ros::Rate loop_rate(10);
  if(flag==0)
  {
    if(firstrun==true)
    {
      sendQuad(1, 0, 'n', -7.5, 1, 2);
      while(ros::ok() && mvpose.reached!='n')
        ros::spinOnce();
      while(ros::ok() && !(mvpose.reached=='y'))
      	ros::spinOnce();
      *frameX = -7.5;
      *frameY = 1;
    }
    else
    {
      sendQuad(1, 0, 'n', -7.5, 9, 2);
      while(mvpose.reached!='n')
        ros::spinOnce();
      while(ros::ok() && !(mvpose.reached=='y'))
      	ros::spinOnce();
      *frameX = -7.5;
      *frameY = 9;
      thresx = 2.5;
      thresy = -1;
    }
    firstrun = false;
    flag = 1;
  }
  int ID = 0;
  ID = findBotNearestToQuad(0);

  ROS_INFO("Found inside frame bot ID: %d\n", ID);

  if(ID==0)
  {
    translateFrame();
    sendQuad(1, 0,'n', *frameX, *frameY, 2);
    while(mvpose.reached!='n')
      ros::spinOnce();
    while(ros::ok() && !(mvpose.reached=='y'))
      ros::spinOnce();
  }

  else
  {
    lockID = ID;

    ROS_INFO("LOCKED ID: %d\n", ID);

    whereToTurn(ID);
    removeTheLockedBot(lockID);
  }
  loop_rate.sleep();
}

int strategyb::removeTheLockedBot(int ID)
{
  int removed = 0;
  bool rotating = false;
  while(ros::ok() && removed==0)
  {
    ros::Rate loop_rate(10);
    nav_msgs::Odometry vel;

    sendQuad(ID, -1, 'n', 0, 0, 0);
    while(mvpose.reached!='n')
      ros::spinOnce();

    ros::spinOnce();
    retrieve_pose(&vel);

    if(fabs(vel.twist.twist.linear.x)<=0.1 && fabs(vel.twist.twist.linear.y)<=0.1)
    {
      while(fabs(vel.twist.twist.linear.x)<=0.01 && fabs(vel.twist.twist.linear.y)>=0.01)
      {
        ROS_INFO("Rotating\n");
        ros::spinOnce();
        retrieve_pose(&vel);
      }
      whereToTurn(ID);
    }
    removed = isOutsideGreen(ID);
  }
  flag = 0;
}

int strategyb::isOutsideGreen(int ID)
{
  nav_msgs::Odometry position;
  ClosestBot.clear();
  ros::Rate loop_rate(10);
  ros::spinOnce();
  retrieve_pose(&position);
  if(position.pose.pose.position.y < 10 && position.pose.pose.position.y > -10 && position.pose.pose.position.x < 10 && position.pose.pose.position.x > -10)
    return 0;
  else
    return 1;
}

int strategyb::translateFrame()
{
  if((*frameX==-7.5 && left==false) || right==true)
  {
    right = true;
    left = false;

    if(*frameY<10 && *frameY>=0)
    {
      if(*frameX==7.5)
        up = true;
      if(up==true)
      {
        *frameY += thresy;
        up = false;
      }
      *frameX = *frameX + thresx;
      if(*frameX>7.5)
      {
        *frameX = *frameX - thresx;
        right = false;
        return 0;
      }
      return 0;
    }
    else
    {
      sendQuad(1, 0, 'n', -7.5, 9, 2);
      while(mvpose.reached!='n')
        ros::spinOnce();
      while(ros::ok() && !(mvpose.reached=='y'))
      	ros::spinOnce();
      thresy = thresy*(-1);
      *frameX = -7.5;
      *frameY = 9;

    }
  }

  if((*frameX==7.5 && right==false) || left==true)
  {
    right = false;
    left = true;
    if(*frameY<10 && *frameY>=0)
    {
      if(*frameX==-7.5)
        up = true;
      if(up==true)
      {
        *frameY += thresy;
        up = false;
      }
      *frameX = *frameX - thresx;
      if(*frameX<-7.5)
      {
        *frameX = *frameX + thresx;
        left = false;
        return 0;
      }
      return 0;
    }
    else
    {
      sendQuad(1, 0, 'n', -7.5, 9, 2);
      while(mvpose.reached!='n')
        ros::spinOnce();
      while(ros::ok() && !(mvpose.reached=='y'))
      	ros::spinOnce();
      thresy = thresy*(-1);
      *frameX = -7.5;
      *frameY = 9;
    }
  }
}

void strategyb::sendQuad(int id, int mode, char reached, double x, double y, double z)
{
  strategy::navigate_quad temp;
  temp.id = id;
  temp.mode = mode;
  temp.reached = reached;
  temp.x = x;
  temp.y = y;
  temp.z = z;
  botToTap.publish(temp);
}

void strategyb::whereToTurn(int ID)
{
  nav_msgs::Odometry position;
  qt positionq;
  double yaw=0, pitch=0, roll=0;
  char publish_name[40];

  sprintf(publish_name, "robot%d/cmd_vel", ID);

  ros::Rate loop_rate(10);
  ros::spinOnce();
  loop_rate.sleep();

  retrieve_pose(&position);

  positionq.x = position.pose.pose.orientation.x;
  positionq.y = position.pose.pose.orientation.y;
  positionq.z = position.pose.pose.orientation.z;
  positionq.w = position.pose.pose.orientation.w;

  double theta1 = atan((10 - position.pose.pose.position.y)/(10 - position.pose.pose.position.x));
	double theta2 = PI + atan((10 - position.pose.pose.position.y)/(-10 - position.pose.pose.position.x));

  GetEulerAngles(positionq, &yaw, &pitch, &roll);

  if(yaw>=theta1 && yaw<=theta2 || (theta1*theta2<0 && (yaw>=theta1 || yaw <=theta2)))
		ROS_INFO("Condition 0 for target bot::::::do nothing\n");

	else if((yaw>=angle(theta1+PI) && yaw<=angle(theta2+PI) ) || (angle(theta1+PI)*angle(theta2+PI)<0 && (yaw>=angle(theta1+PI) || yaw<=angle(theta2+PI)) ) )
	{
		ROS_INFO("Condition 1 for target bot: 180 degree turn %f\n",yaw);

    sendQuad(ID, 0, 'n', 0, 0, 0);
    while(mvpose.reached!='n')
      ros::spinOnce();
    while(ros::ok() && !(mvpose.reached=='y'))
      ros::spinOnce();
	}
	else if((yaw>theta2 && yaw<angle(theta2+PI/4)) || ( theta2*angle(theta2+PI/4)<0 && (yaw>theta2 || yaw<angle(theta2+PI/4))))
	{
  	ROS_INFO("Condition 2 for target bot: 180 then 45 degree turn\n");

    sendQuad(ID, 0, 'n', 0, 0, 0);
    while(mvpose.reached!='n')
      ros::spinOnce();
    while(ros::ok() && !(mvpose.reached=='y'))
      ros::spinOnce();

    sendQuad(ID, 1, 'n', 0, 0, 0);
    while(mvpose.reached!='n')
      ros::spinOnce();
    while(ros::ok() && !(mvpose.reached=='y'))
      ros::spinOnce();
	}
	else if(yaw>=angle(theta2 + PI/4) && yaw<=angle(theta2 + PI/2) || ( angle(theta2+PI/4)*angle(theta2+PI/2)<0 && (yaw>angle(theta1+PI/4) || yaw<angle(theta2+PI/2))))
	{
		ROS_INFO("Condition 3 for target bot: 90 degree turn, tap twice\n");

    sendQuad(ID, 1, 'n', 0, 0, 0);
    while(mvpose.reached!='n')
      ros::spinOnce();
    while(ros::ok() && !(mvpose.reached=='y'))
      ros::spinOnce();

    sendQuad(ID, 1, 'n', 0, 0, 0);
    while(mvpose.reached!='n')
      ros::spinOnce();
    while(ros::ok() && !(mvpose.reached=='y'))
      ros::spinOnce();
	}
	else if(yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4) || (angle(theta2+PI)*angle(theta2+PI+PI/4) && (yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4))))
	{
		ROS_INFO("Condition 4 for target bot: 45 degree turn \n");

    sendQuad(ID, 1, 'n', 0, 0, 0);
    while(mvpose.reached!='n')
      ros::spinOnce();
    while(ros::ok() && !(mvpose.reached=='y'))
      ros::spinOnce();
	}
}

float strategyb::angle(float ang){
  if(ang>=-PI && ang<=PI)
    return ang;
  else if(ang>PI)
    return angle(ang-2*PI);
  else if(ang<-PI)
    return angle(ang+2*PI);
}

void strategyb::GetEulerAngles(qt q, double* yaw, double* pitch, double* roll)
 {
     const double w2 = q.w*q.w;
     const double x2 = q.x*q.x;
     const double y2 = q.y*q.y;
     const double z2 = q.z*q.z;
     const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
     const double abcd = q.w*q.x + q.y*q.z;
     const double eps = 1e-7;    // TODO: pick from your math lib instead of hardcoding.
     const double pi = 3.14159265358979323846;   // TODO: pick from your math lib instead of hardcoding.
     if (abcd > (0.5-eps)*unitLength)
     {
         *yaw = 2 * atan2(q.y, q.w);
         *pitch = pi;
         *roll = 0;
     }
     else if (abcd < (-0.5+eps)*unitLength)
     {
         *yaw = -2 * ::atan2(q.y, q.w);
         *pitch = -pi;
         *roll = 0;
     }
     else
     {
         const double adbc = q.w*q.z - q.x*q.y;
         const double acbd = q.w*q.y - q.x*q.z;
         *yaw = ::atan2(2*adbc, 1 - 2*(z2+x2));
         *pitch = ::asin(2*abcd/unitLength);
         *roll = ::atan2(2*acbd, 1 - 2*(y2+x2));
     }
 }

void strategyb::groundbotCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  bot_pose.pose.pose.position.x = msg->pose.pose.position.x;
  bot_pose.pose.pose.position.y = msg->pose.pose.position.y;
  bot_pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  bot_pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  bot_pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  bot_pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  bot_pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  bot_pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}


void strategyb::feedbackfn(const nav_msgs::Odometry::ConstPtr& odom_data)
{
  MAVpose = *odom_data;
}

void strategyb::nav_quad_callback(const strategy::navigate_quad::ConstPtr& msg)
{
  mvpose = *msg;
}


void strategyb::retrieve_pose(nav_msgs::Odometry *gbpose)
{
  gbpose->pose.pose.orientation.x = bot_pose.pose.pose.orientation.x;
  gbpose->pose.pose.orientation.y = bot_pose.pose.pose.orientation.y;
  gbpose->pose.pose.orientation.z = bot_pose.pose.pose.orientation.z;
  gbpose->pose.pose.orientation.w = bot_pose.pose.pose.orientation.w;
  gbpose->pose.pose.position.x = bot_pose.pose.pose.position.x;
  gbpose->pose.pose.position.y = bot_pose.pose.pose.position.y;
  gbpose->pose.pose.position.z = bot_pose.pose.pose.position.z;
  gbpose->twist.twist.linear.x = bot_pose.twist.twist.linear.x;
  gbpose->twist.twist.linear.y = bot_pose.twist.twist.linear.y;
  gbpose->twist.twist.linear.z = bot_pose.twist.twist.linear.z;
}
