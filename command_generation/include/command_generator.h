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
#include<wheel_control/wheel_msg.h>

class command_generator {
    private:
        //ros publisher subscriber
        ros::NodeHandle nhp1,nhp2,nhs,nhs2;
        ros::Publisher pub1,pub2;
        ros::Subscriber sub,sub2;
        ros::CallbackQueue queue,queue2;

        //ros msgs
        command_generation::filted_objects_info obj_info;
        command_generation::robot_odm robot_odm;
        command_generation::robot_odm goal_odm;
        float vrt,wrt;
    	const float f=350.505;
		const int width=672;
		const int height=376;
        //dicriminate_obstacle
        std::vector<int> obstacle_status;
    public:
        command_generator();
        ~command_generator();
        //setting
        bool setting_RobotExpCondition(APF_MPC& apf_mpc,float reso);
        bool update_RobotPos(APF_MPC& apf_mpc);
        void update_RobotVel(float& v,float& w);
        float& get_vel(void);
        float& get_angVel(void);
        void set_pos(cv::Point2f& x);

        //subscribe
        void subscribe_objects(void);
        void objects_callback(const command_generation::filted_objects_info::ConstPtr& msg);
        void subscribe_odometry(void);
        void odometry_callback(const command_generation::robot_odm::ConstPtr& msg);
        //set obstacle status
        bool dicriminate_obstacle(void);
        //set obstacle data
        void set_obstacles(APF_MPC& apf_mpc);
        //publish
        void publish_velocity(float& v,float w);
        void publish_wheel_velocity(float& v,float w);

    };
#endif