#include <iostream>
#include <math.h>
#include <utility>
#include <set>
#include <math.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "../include/strategy/strategyb.h"

#define PI 3.14159
#define angular_speed 0.76
using namespace std;

int strategyb::findBotNearestToQuad()
{
  nav_msgs::Odometry position;
  ClosestBot.clear();
  ros::Rate loop_rate(10);
  ros::spinOnce();
  loop_rate.sleep();
  int i;
  while(ClosestBot.empty())
  {
    ROS_INFO("finding closest bot\n");
    ClosestBot.clear();
    for(i=4; i<=13; i++)
    {
      ros::Rate loop_rate(10);
      ros::spinOnce();
      loop_rate.sleep();
      retrieve_pose(i, &position);
      if(abs(MAVpose.pose.pose.position.x - position.pose.pose.position.x)<=5 && abs(MAVpose.pose.pose.position.y - position.pose.pose.position.y)<=2)
      {
        double distance = sqrt(pow(MAVpose.pose.pose.position.x - position.pose.pose.position.x,2) - pow(MAVpose.pose.pose.position.y - position.pose.pose.position.y,2));
        ClosestBot.insert(make_pair(distance, i));
      }
    }
  }
  return ((*ClosestBot.begin()).second);
}

void strategyb::initialHerd()
{
  sendQuad(1, 0, 'n', 0, -4, 2);
  ROS_INFO("Going to 0,-4 m\n");
  ros::Rate loop_rate(10);
  ros::spinOnce();

  while(ros::ok() && !(mvpose.reached=='y'))
  {
  	ROS_INFO("checking for y\n");
  	ros::spinOnce();
  }

  int ID = findBotNearestToQuad();
  ROS_INFO("Found nearest bot %d\n", ID);

  sendQuad(ID, 1, 'n', 0, 0, 0);
  while(mvpose.reached!='n')
    ros::spinOnce();
  //ROS_INFO("N OR Y %c\n", mvpose.reached);
  while(ros::ok() && !(mvpose.reached=='y'))
  {
  	//ROS_INFO("checking for y AGAIN\n");
  	ros::spinOnce();
  }

  loop_rate.sleep();
  whereToTurn(ID);
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

  retrieve_pose(ID, &position);

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
		//ROS_INFO("Condition 1 for target bot::::::180 degree turn %f\n",yaw);
    rotate(PI,publish_name,ID);
	}
	else if((yaw>theta2 && yaw<angle(theta2+PI/4)) || ( theta2*angle(theta2+PI/4)<0 && (yaw>theta2 || yaw<angle(theta2+PI/4))))
	{
    	//ROS_INFO("Condition 4 for target bot::::::180 then 45 degree turn\n");
    	rotate((5*PI/4),publish_name,ID);
	}
	else if(yaw>=angle(theta2 + PI/4) && yaw<=angle(theta2 + PI/2) || ( angle(theta2+PI/4)*angle(theta2+PI/2)<0 && (yaw>angle(theta1+PI/4) || yaw<angle(theta2+PI/2))))
	{
		//ROS_INFO("Condition 3 for target bot::::::90 degree turn, tap twice\n");
    rotate((PI/2),publish_name,ID);
	}
	else if(yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4) || (angle(theta2+PI)*angle(theta2+PI+PI/4) && (yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4))))
	{
		//ROS_INFO("Condition 2 for target bot::::::45 degree turn \n");
    rotate((PI/4),publish_name,ID);
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

void strategyb::rotate (double relative_angle, char publish_name[40], int ID)
{
  ROS_INFO("ROTATING\n");
	geometry_msgs::Twist vel_msg;
  nav_msgs::Odometry temp;
  qt q;
  double Yaw,Pitch,Roll,yaw_i,pitch_i,roll_i;
  Yaw=Pitch=Roll= 0.0;
  publi = n.advertise<geometry_msgs::Twist>(publish_name,1000);
  //set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
  vel_msg.linear.y =0;
  vel_msg.linear.z =0;
  //set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z =-abs(angular_speed);

  ros::Rate loop_rate(100);
  ros::spinOnce();
  retrieve_pose(ID, &temp);
  loop_rate.sleep();

  q.x = temp.pose.pose.orientation.x;
  q.y = temp.pose.pose.orientation.y;
  q.z = temp.pose.pose.orientation.z;
  q.w = temp.pose.pose.orientation.w;

  GetEulerAngles(q, &yaw_i, &pitch_i, &roll_i);

  while(1)
  {
    //ROS_INFO("STILL ROTATING\n");
    ros::spinOnce();
    retrieve_pose(ID, &temp);
    loop_rate.sleep();
    qt q;
    q.x = temp.pose.pose.orientation.x;
    q.y = temp.pose.pose.orientation.y;
    q.z = temp.pose.pose.orientation.z;
    q.w = temp.pose.pose.orientation.w;
    GetEulerAngles(q, &Yaw, &Pitch, &Roll);
    publi.publish(vel_msg);
    if(fabs(angle(Yaw) - angle(yaw_i+relative_angle)) <= 0.1)
  	 break;
   }
 vel_msg.angular.z = 0;
 publi.publish(vel_msg);
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

void strategyb::groundbot4Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb4pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb4pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb4pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb4pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb4pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb4pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb4pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb4pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategyb::groundbot5Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb5pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb5pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb5pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb5pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb5pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb5pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb5pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb5pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategyb::groundbot6Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb6pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb6pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb6pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb6pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb6pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb6pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb6pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb6pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategyb::groundbot7Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb7pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb7pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb7pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb7pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb7pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb7pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb7pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb7pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategyb::groundbot8Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb8pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb8pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb8pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb8pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb8pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb8pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb8pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb8pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategyb::groundbot9Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb9pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb9pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb9pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb9pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb9pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb9pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb9pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb9pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategyb::groundbot10Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb10pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb10pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb10pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb10pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb10pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb10pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb10pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb10pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategyb::groundbot11Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb11pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb11pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb11pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb11pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb11pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb11pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb11pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb11pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategyb::groundbot12Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb12pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb12pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb12pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb12pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb12pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb12pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb12pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb12pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategyb::groundbot13Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb13pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb13pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb13pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb13pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb13pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb13pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb13pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb13pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategyb::feedbackfn(const nav_msgs::Odometry::ConstPtr& odom_data)
{
  // MAVpose.pose.pose.position.x = odom_data->pose.pose.position.x;
  // MAVpose.pose.pose.position.y = odom_data->pose.pose.position.y;
  // MAVpose.pose.pose.orientation.x = odom_data->pose.pose.orientation.x;
  // MAVpose.pose.pose.orientation.y = odom_data->pose.pose.orientation.y;
  // MAVpose.pose.pose.orientation.z = odom_data->pose.pose.orientation.z;
  // MAVpose.pose.pose.orientation.w = odom_data->pose.pose.orientation.w;
  // MAVpose.twist.twist.linear.x = odom_data->twist.twist.linear.x;
  // MAVpose.twist.twist.linear.y = odom_data->twist.twist.linear.y;
  MAVpose = *odom_data;
}

void strategyb::nav_quad_callback(const strategy::navigate_quad::ConstPtr& msg)
{
  mvpose = *msg;
}


void strategyb:: retrieve_pose(int ID, nav_msgs::Odometry *gbpose)
{
  switch(ID)
  {
    case 4:
      gbpose->pose.pose.orientation.x = gb4pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb4pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb4pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb4pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb4pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb4pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb4pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb4pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb4pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb4pose.twist.twist.linear.z;
      //return gbpose;
      break;

    case 5:
      gbpose->pose.pose.orientation.x = gb5pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb5pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb5pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb5pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb5pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb5pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb5pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb5pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb5pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb5pose.twist.twist.linear.z;
     // return gbpose;
      break;

    case 6:
      gbpose->pose.pose.orientation.x = gb6pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb6pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb6pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb6pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb6pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb6pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb6pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb6pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb6pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb6pose.twist.twist.linear.z;
     // return gbpose;
      break;
    case 7:
      gbpose->pose.pose.orientation.x = gb7pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb7pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb7pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb7pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb7pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb7pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb7pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb7pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb7pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb7pose.twist.twist.linear.z;
     // return gbpose;
      break;

    case 8:
      gbpose->pose.pose.orientation.x = gb8pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb8pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb8pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb8pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb8pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb8pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb8pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb8pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb8pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb8pose.twist.twist.linear.z;
     // return gbpose;
      break;

    case 9:
      gbpose->pose.pose.orientation.x = gb9pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb9pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb9pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb9pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb9pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb9pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb9pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb9pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb9pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb9pose.twist.twist.linear.z;
     // return gbpose;
      break;

    case 10:
      gbpose->pose.pose.orientation.x = gb10pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb10pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb10pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb10pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb10pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb10pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb10pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb10pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb10pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb10pose.twist.twist.linear.z;
      //return gbpose;
      break;

    case 11:
      gbpose->pose.pose.orientation.x = gb11pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb11pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb11pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb11pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb11pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb11pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb11pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb11pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb11pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb11pose.twist.twist.linear.z;
      //return gbpose;
      break;

    case 12:
      gbpose->pose.pose.orientation.x = gb12pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb12pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb12pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb12pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb12pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb12pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb12pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb12pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb12pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb12pose.twist.twist.linear.z;
      //return gbpose;
      break;

    case 13:
      gbpose->pose.pose.orientation.x = gb13pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb13pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb13pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb13pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb13pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb13pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb13pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb13pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb13pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb13pose.twist.twist.linear.z;
      //return gbpose;
      break;
  }
}
