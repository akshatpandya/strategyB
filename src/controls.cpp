// topic name : groundbot/tap
// message type - strategy/navigate_quad
// Header header
// int32 id
// int32 mode
// char reached
// float64 x
// float64 y
// float64 z
// 1 for tap in int32 mode and 0 for bump(land in front for 180 degree turn)

#include <ros/ros.h>
#include <time.h>
#include <math.h>
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "strategy/navigate_quad.h" //message type for strategy with groundbot/tap topic

#define step 0.1              // step for changing altitude gradually
#define Eps 0.2             // range for error
#define Default 1          // Default height for the quad
#define Delay 5             // time duration for which it is idle in front of the ground bot
#define GBHeight 0.2
#define Epsz 0.1

float t0 = 3;                                                  //descent time
//float Eps;                                                    //Error margin
double theta;                                                  //orientation of gb wrt X-axis
float ErrorLin;
float ErrorLin_ObsMAV,ErrorQuad,ErrorObs ;
float obs_theta;
float set_theta;
strategy::navigate_quad QuadStatus;
nav_msgs::Odometry obspose;
nav_msgs::Odometry gbpose;                                      //Get linear error
nav_msgs::Odometry gb4pose;                                     //position of ground bot
nav_msgs::Odometry gb5pose;
nav_msgs::Odometry gb6pose;
nav_msgs::Odometry gb7pose;
nav_msgs::Odometry gb8pose;
nav_msgs::Odometry gb9pose;
nav_msgs::Odometry gb10pose;
nav_msgs::Odometry gb11pose;
nav_msgs::Odometry gb12pose;
nav_msgs::Odometry gb13pose;
nav_msgs::Odometry MAVpose;                                        //position of quad
nav_msgs::Odometry MAVdest;                                         //quad destination
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped pose0;
geometry_msgs::PoseStamped pose1;
geometry_msgs::PoseStamped pose2;
geometry_msgs::PoseStamped pose3;
geometry_msgs::PoseStamped pose4;
strategy::navigate_quad interact;
int flag = 0;
int check = 0;
int count = 0;
int status;
double yaw, pitch, roll;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


struct Quaternionm
{
    double w, x, y, z;
};


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
void feedbackfn(const nav_msgs::Odometry::ConstPtr& odom_data);
void obsCallback(const nav_msgs::Odometry::ConstPtr& msg);
void StatusCallback(const strategy::navigate_quad::ConstPtr& msg);
float GetErrorLin(const nav_msgs::Odometry MAVdest ,const nav_msgs::Odometry MAVpose);
void GetEulerAngles(Quaternionm q, double* yaw, double* pitch, double* roll);
double GetTheta();
void follow();
void descent();
void ascent();
double kp;

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"controls");

  kp=0.08;//0.08

  ros::NodeHandle n;
  ros::Subscriber imu_yaw = n.subscribe("mavros/local_position/odom", 10, feedbackfn);
  ros::Subscriber gbpose_sub_4 = n.subscribe("/robot4/odom", 100, groundbot4Callback); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_5 = n.subscribe("/robot5/odom", 100, groundbot5Callback); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_6 = n.subscribe("/robot6/odom", 100, groundbot6Callback); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_7 = n.subscribe("/robot7/odom", 100, groundbot7Callback); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_8 = n.subscribe("/robot8/odom", 100, groundbot8Callback); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_9 = n.subscribe("/robot9/odom", 100, groundbot9Callback); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_10 = n.subscribe("/robot10/odom", 100, groundbot10Callback); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_11 = n.subscribe("/robot11/odom", 100, groundbot11Callback); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_12 = n.subscribe("/robot12/odom", 100, groundbot12Callback); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_13 = n.subscribe("/robot13/odom", 100, groundbot13Callback); // subscriber to get ground bot position
  ros::Subscriber obspose_sub = n.subscribe("/robot3/odom", 100, obsCallback);
  ros::Subscriber Status_sub= n.subscribe("groundbot/tap", 10, StatusCallback);
  ros::Publisher Status_pub = n.advertise<strategy::navigate_quad>("groundbot/tap", 10);
  ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
                      ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
                      ("mavros/set_mode");

  ros::Rate rate(20.0);

  // wait for FCU connection
  while(ros::ok() && current_state.connected)
  {
          ros::spinOnce();
          rate.sleep();
  }


    geometry_msgs::PoseStamped pose0;
    pose0.pose.position.x = 0;
    pose0.pose.position.y = 0;
    pose0.pose.position.z = Default;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose0);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();


  while(ros::ok())
  {

    if( current_state.mode != "OFFBOARD" &&   (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
                {
                ROS_INFO("Offboard enabled");
                }
            last_request = ros::Time::now();
    }
    else
    {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                    {
                    ROS_INFO("Vehicle armed");
                    }
                last_request = ros::Time::now();
                }
    }

    if(QuadStatus.id == 1)
    {
      pose4.pose.position.x = QuadStatus.x;
      pose4.pose.position.y = QuadStatus.y;
      pose4.pose.position.z = QuadStatus.z;
    }



    theta = GetTheta();
    if(QuadStatus.mode == 0)
    {
      MAVdest.pose.pose.position.x = gbpose.pose.pose.position.x + ((t0)*(gbpose.twist.twist.linear.x) + 1)*(cos(theta));
      MAVdest.pose.pose.position.y = gbpose.pose.pose.position.y +  ((t0)*(gbpose.twist.twist.linear.x)+1)*(sin(theta));
    }
    else if(QuadStatus.mode == 1)
    {
      MAVdest.pose.pose.position.x = gbpose.pose.pose.position.x + (t0)*(gbpose.twist.twist.linear.x)*(cos(theta));
      MAVdest.pose.pose.position.y = gbpose.pose.pose.position.y +  (t0)*(gbpose.twist.twist.linear.x)*(sin(theta));
    }

    ErrorLin = GetErrorLin(MAVdest,MAVpose);                       //error between expected and actual position of quad



    if(ErrorLin > Eps)
    {
	    if(MAVpose.pose.pose.position.z-Default<=Epsz)
	    {
		follow();
		if(status==1) local_pos_pub.publish(pose1);
		else if(status==2) local_pos_pub.publish(pose2);
	    }

	    else
	    {
	    ascent();
	    local_pos_pub.publish(pose);
	    }
    }
    else  if (ErrorLin <= Eps)
    {
      if(count!=0)
      {
      ROS_INFO("ok");
     if(flag == 0)
      {
        if(ErrorLin_ObsMAV <= 2)
        {
          pose3.pose.position.x = MAVpose.pose.pose.position.x;
          pose3.pose.position.y = MAVpose.pose.pose.position.y;
          pose3.pose.position.z = Default;
          local_pos_pub.publish(pose3);
        }
        else
        {
	      descent();
        local_pos_pub.publish(pose);
        }
      }
      //printf("flag=%d\n", flag);
       if (fabs(MAVpose.pose.pose.position.z-GBHeight)<=Epsz)
      {
	ROS_INFO("YO\n YO\n YO\n YO\nYO\nYO\nYO\nYO\nYO\nYO\nYO\nYO\nYO\nYO\nYO\n");
      ascent();
      local_pos_pub.publish(pose);
      flag = 1;
      }
	else if(flag ==1)
	{   ROS_INFO("avoiding RTL\n");
	    pose.pose.position.x = MAVpose.pose.pose.position.x;
	    pose.pose.position.y = MAVpose.pose.pose.position.y;
	    pose.pose.position.z = Default;
	    local_pos_pub.publish(pose);
      QuadStatus.reached = 'y';
      Status_pub.publish(QuadStatus);
	}
        }

      count ++;
    }

    ros::spinOnce();


  }
  return (0);
}

void StatusCallback(const strategy::navigate_quad::ConstPtr& msg)
{
QuadStatus.id = msg->id;
QuadStatus.mode = msg->mode;
QuadStatus.reached = msg->reached;
QuadStatus.x = msg->x;
QuadStatus.y = msg->y;
QuadStatus.z = msg->z;
  return;
}

void groundbot4Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void groundbot5Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void groundbot6Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void groundbot7Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void groundbot8Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void groundbot9Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void groundbot10Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void groundbot11Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void groundbot12Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void groundbot13Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void obsCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  obspose.pose.pose.position.x = msg->pose.pose.position.x;
  obspose.pose.pose.position.y = msg->pose.pose.position.y;
  obspose.pose.pose.position.z = msg->pose.pose.position.z;
  return;
}

void feedbackfn(const nav_msgs::Odometry::ConstPtr& odom_data)
{
    MAVpose= *odom_data;
}




float GetErrorLin(const nav_msgs::Odometry MAVdest ,const nav_msgs::Odometry MAVpose)
{
  float El;
  El = sqrt(pow((MAVdest.pose.pose.position.x - MAVpose.pose.pose.position.x),2) + pow((MAVdest.pose.pose.position.y - MAVpose.pose.pose.position.y),2));
  return(El);
}

void GetEulerAngles(Quaternionm q, double* yaw, double* pitch, double* roll)
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

   void follow()
   {
     double z;
  /*  pose.pose.position.x = MAVdest.pose.pose.position.x;
    pose.pose.position.y = MAVdest.pose.pose.position.y;
    pose.pose.position.z = Default;*/
     //ROS_INFO("%f \t %f \t palak ", ErrorLin, MAVpose.pose.pose.position.z );

     obs_theta= atan2((MAVpose.pose.pose.position.y-(obspose.pose.pose.position.y)),(MAVpose.pose.pose.position.x-(obspose.pose.pose.position.x)));
     set_theta= atan2(MAVdest.pose.pose.position.y-(obspose.pose.pose.position.y),MAVdest.pose.pose.position.x-(obspose.pose.pose.position.x));

     ErrorLin_ObsMAV=GetErrorLin(obspose,MAVpose);
    // ErrorQuad = sqrt(pow((y -(current_y)),2) + pow((x - (current_x)),2));
    // ErrorObs = sqrt(pow((y - (obspose.pose.pose.position.y)),2) + pow((x - (obspose.pose.pose.position.x)),2));
     ErrorObs = GetErrorLin(obspose, MAVdest);
     ErrorQuad = GetErrorLin(MAVdest, MAVpose);


     if(ErrorLin_ObsMAV < 2 && (ErrorObs<ErrorQuad))
     {
          ROS_INFO("%f ----- %f,%f  aditi",ErrorLin_ObsMAV,(set_theta),(obs_theta));
         //ROS_INFO("%f %f %f \n",current_x,current_y,current_z);
         //ROS_INFO("%f", ErrorQuad);
         if((set_theta - obs_theta) > 3.14)
           {
             z=kp*(6.28 - (set_theta - obs_theta));
           }
         else if( (set_theta - obs_theta) < (-3.14))
           {
             z=kp*((set_theta - obs_theta)+6.28);
           }
 	else
 	{
  	 z=kp*(set_theta - obs_theta);
 	}

         obs_theta+=z;

                 // dest.false_dest(((current_y)*(-1)+2*cos(set_theta)),(current_x+2*sin(set_theta)),Default,0);
     pose1.pose.position.x = ((obspose.pose.pose.position.x)+ (2)*cos(obs_theta));
     pose1.pose.position.y = ((obspose.pose.pose.position.y)+ (2)*sin(obs_theta));
     pose1.pose.position.z = Default;
     /*local_pos_pub.publish(pose1);*/
     status=1;
     }

     else
     {
	ROS_INFO("%f ----- %f,%f  follow",ErrorLin_ObsMAV,(set_theta),(obs_theta));
     pose2.pose.position.x = MAVdest.pose.pose.position.x;
     pose2.pose.position.y = MAVdest.pose.pose.position.y;
     pose2.pose.position.z = Default;
     /*local_pos_pub.publish(pose1);*/
     status=2;
     }

   }

   void descent()
  {
  MAVdest.pose.pose.position.z =GBHeight;                                   //descent of MAV
   ROS_INFO("%f \t %f \t palak descent", ErrorLin, MAVpose.pose.pose.position.z );
  //destination.set_dest((MAVpose.pose.pose.position.y)*(-1),MAVpose.pose.pose.position.x,MAVdest.pose.pose.position.z,0);
  pose.pose.position.x = MAVdest.pose.pose.position.x;
  pose.pose.position.y = MAVdest.pose.pose.position.y;
  pose.pose.position.z = MAVdest.pose.pose.position.z;
  //local_pos_pub.publish(pose);
  ROS_INFO("%f \t %f \t  ",ErrorLin, MAVpose.pose.pose.position.z );
  }

  void ascent()
  {
    //destination.set_dest((MAVdest.pose.pose.position.y)*(-1),MAVdest.pose.pose.position.x,Default,0);
     ROS_INFO("%f \t %f \t palak ascent", ErrorLin, MAVpose.pose.pose.position.z );
    pose.pose.position.x = MAVpose.pose.pose.position.x;
    pose.pose.position.y = MAVpose.pose.pose.position.y;
    pose.pose.position.z = Default;
    //local_pos_pub.publish(pose);
    ROS_INFO("%f \t %f \t  ",ErrorLin, MAVpose.pose.pose.position.z );
  }

  double GetTheta()
  {
    Quaternionm myq;

    if(QuadStatus.id == 4)
    {
     myq.x = gb4pose.pose.pose.orientation.x;
     myq.y = gb4pose.pose.pose.orientation.y;
     myq.z = gb4pose.pose.pose.orientation.z;
     myq.w = gb4pose.pose.pose.orientation.w;

     gbpose.pose.pose.position.x = gb4pose.pose.pose.position.x;
     gbpose.pose.pose.position.y = gb4pose.pose.pose.position.y;
     gbpose.pose.pose.position.z = gb4pose.pose.pose.position.z;
     gbpose.twist.twist.linear.x = gb4pose.twist.twist.linear.x;
     gbpose.twist.twist.linear.y = gb4pose.twist.twist.linear.y;
     gbpose.twist.twist.linear.z = gb4pose.twist.twist.linear.z;
    }

    else if (QuadStatus.id == 5)
    {
      myq.x = gb5pose.pose.pose.orientation.x;
      myq.y = gb5pose.pose.pose.orientation.y;
      myq.z = gb5pose.pose.pose.orientation.z;
      myq.w = gb5pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb5pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb5pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb5pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb5pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb5pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb5pose.twist.twist.linear.z;
    }

    else if (QuadStatus.id == 6)
    {
      myq.x = gb6pose.pose.pose.orientation.x;
      myq.y = gb6pose.pose.pose.orientation.y;
      myq.z = gb6pose.pose.pose.orientation.z;
      myq.w = gb6pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb6pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb6pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb6pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb6pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb6pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb6pose.twist.twist.linear.z;
    }

    else if (QuadStatus.id == 7)
    {
      myq.x = gb7pose.pose.pose.orientation.x;
      myq.y = gb7pose.pose.pose.orientation.y;
      myq.z = gb7pose.pose.pose.orientation.z;
      myq.w = gb7pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb7pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb7pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb7pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb7pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb7pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb7pose.twist.twist.linear.z;
    }

    else if (QuadStatus.id == 8)
    {
      myq.x = gb8pose.pose.pose.orientation.x;
      myq.y = gb8pose.pose.pose.orientation.y;
      myq.z = gb8pose.pose.pose.orientation.z;
      myq.w = gb8pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb8pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb8pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb8pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb8pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb8pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb8pose.twist.twist.linear.z;
    }

    else if (QuadStatus.id == 9)
    {
      myq.x = gb9pose.pose.pose.orientation.x;
      myq.y = gb9pose.pose.pose.orientation.y;
      myq.z = gb9pose.pose.pose.orientation.z;
      myq.w = gb9pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb9pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb9pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb9pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb9pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb9pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb9pose.twist.twist.linear.z;
    }

    else if (QuadStatus.id == 10)
    {
      myq.x = gb10pose.pose.pose.orientation.x;
      myq.y = gb10pose.pose.pose.orientation.y;
      myq.z = gb10pose.pose.pose.orientation.z;
      myq.w = gb10pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb10pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb10pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb10pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb10pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb10pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb10pose.twist.twist.linear.z;
    }

    else if (QuadStatus.id == 11)
    {
      myq.x = gb11pose.pose.pose.orientation.x;
      myq.y = gb11pose.pose.pose.orientation.y;
      myq.z = gb11pose.pose.pose.orientation.z;
      myq.w = gb11pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb11pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb11pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb11pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb11pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb11pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb11pose.twist.twist.linear.z;
    }

    else if (QuadStatus.id == 12)
    {
      myq.x = gb12pose.pose.pose.orientation.x;
      myq.y = gb12pose.pose.pose.orientation.y;
      myq.z = gb12pose.pose.pose.orientation.z;
      myq.w = gb12pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb12pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb12pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb12pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb12pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb12pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb12pose.twist.twist.linear.z;
    }

    else if (QuadStatus.id == 13)
    {
      myq.x = gb13pose.pose.pose.orientation.x;
      myq.y = gb13pose.pose.pose.orientation.y;
      myq.z = gb13pose.pose.pose.orientation.z;
      myq.w = gb13pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb13pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb13pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb13pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb13pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb13pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb13pose.twist.twist.linear.z;
    }

    GetEulerAngles(myq, &yaw, &pitch, &roll);

    return(yaw);

  }
