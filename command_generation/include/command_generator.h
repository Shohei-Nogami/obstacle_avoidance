#ifndef INCLUDE_COMMAND_GENERATOR_CLASS
#define INCLUDE_COMMAND_GENERATOR_CLASS

#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<command_generation/point3d.h>
#include<command_generation/object_info.h>
#include<command_generation/objects_info.h>
#include<command_generation/filted_objects_info.h>
#include<command_generation/filted_object_info.h>
#include"time_class.h"
#include <pcl_ros/point_cloud.h>
#include<fstream>//file input output
#include<apf_mpc.h>
#include<command_generation/point2d.h>
#include<command_generation/select_theta.h>
#include<command_generation/robot_odm.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Empty.h>
#include<omp.h>
#include<image_transport/image_transport.h>
class command_generator {

    //ros publisher subscriber
    ros::NodeHandle nhp,nhs,nhs2;
    ros::Publisher pub;
    ros::Subscriber sub,sub2;
    ros::CallbackQueue queue,queue2;

    //ros msgs
	command_generation::filted_objects_info obj_info;
    command_generation::robot_odm robot_odm;
    
	//dicriminate_obstacle
	std::vector<int> obstacle_status;
	std::vector<int> obstacle_safety_status;
    //setting
    bool setting_RobotExpCondition(APF_MPC& apf_mpc,float reso);
    //subscribe
    void subscribe_objects(void);
    void objects_callback(const command_generation::filted_objects_info::ConstPtr& msg);
    void subscribe_odometry(void);
    void odometry_callback(const command_generation::robot_odm::ConstPtr& msg);
    //set obstacle status
    bool dicriminate_obstacle(void);
    void clear_safety_status(void);
    //set obstacle data
    void set_obstacles(APF_MPC& apf_mpc);
    //publish
    void publish_velocity(float& v,float w);
}