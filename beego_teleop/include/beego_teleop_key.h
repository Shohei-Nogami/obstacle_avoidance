#include"ros/ros.h"
#include<wheel_control/wheel_msg.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>

	ros::Subscriber sub_twist;
	ros::Publisher pub_wheel;
	int maxvel=600;
	std::string order;
	int vel_r=0;
	int vel_l=0;
	int target_vel_r=0;
	int target_vel_l=0;
	wheel_control::wheel_msg wheelMsg;
	int pbtn=0;
	int cbtn;
	const int ac=50;
	int pvel_r=0;
	int pvel_l=0;
	int T;
	ros::CallbackQueue twist_queue;
	ros::Time start_time;
	double prev_time;	
	double new_time;	//
	double dt;	
