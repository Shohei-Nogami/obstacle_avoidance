#ifndef INCLUDE_AVOID_CLASS
#define INCLUDE_AVOID_CLASS

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
//#include<Eigen/Core>
//#include<Eigen/Dense>
#include<fstream>//file input output
#include<vfh+_class.h>
#include<command_generation/point2d.h>
#include<command_generation/select_theta.h>
#include<command_generation/robot_odm.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Empty.h>
#include<omp.h>
#include<image_transport/image_transport.h>
class avoid {
private:
	ros::NodeHandle nh_sub,nh_sub2, nh_pub,nh_pub2;

	ros::Publisher pub,pub2, pub_pcl, pub_pcl2;
	ros::Subscriber sub,sub2;
	ros::CallbackQueue queue,queue2;

	command_generation::filted_objects_info obj_info;
	command_generation::robot_odm robot_odm;
	cv::Point2f x0 ;
	double theta0;

	std::vector<float> temp_vel;//vel list
  	std::vector<bool> not_select_angle;


	float selected_vel;

	int ROBOT_STATUS;

	//dicriminate_obstacle
	std::vector<int> obstacle_status;
	std::vector<int> obstacle_safety_status;
	vfh_class vfh;
	//vfh parameter
	int min_angle=-45;
	int max_angle=45;
	int vfh_resolution=90;
	float w_target = 0.7;
	float w_angle = 0.5;
	float robot_r=0.2;
	float d_r=0.1;
	float mv_length=0.2;
	float max_vel_dif=0.2;
	//camera param
	static const int width=672;
	static const int height=376;
	const float f=350.505;

	//set_equation
	std::vector<float> obj_eq_a;//ŒX‚«
	//select_route
	int selected_angle_i;
	float selected_angle;
	//set_intersection
	std::vector<cv::Point2f> x_c;
	//set_intersection_time
	std::vector<double> t_c;
	//set_dengerous_time_range
	std::vector<double> t_c_min;
	std::vector<double> t_c_max;
	//check_collision
	std::vector<std::vector<float>> t_c_min_angle;//[angle][vel_num]
	//select_safety_vel
  	int selected_vel_num;
  	//set_safety_vel
	float safety_vel;
	float safety_angle;
	//iecon poster
	cv::Mat depth_image;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	ros::NodeHandle nh_sub_pcl;
	ros::Subscriber sub_pcl;
	ros::CallbackQueue queue_pcl;
	const int H=376;
	const int W=672;
	ros::NodeHandle nh_pub_dimg;
	// ros::Publisher pub_dimg;
	image_transport::ImageTransport it_pub;
	image_transport::Publisher pub_dimg;
public:
	avoid();
	~avoid();
	void set_param_vfh(void);
	void subscribe_objects(void);
	void objects_callback(const command_generation::filted_objects_info::ConstPtr& msg);
	void subscribe_odometry(void);
	void odometry_callback(const command_generation::robot_odm::ConstPtr& msg);
	bool dicriminate_obstacle(void);
	void clear_safety_status(void);
	void set_equation(void);
	void set_grid_map(void);

	void set_not_select_angle_to_vfh(void);

	bool select_route(void);
	void set_intersection(void);
	void set_intersection_time(void);
	void labeling_dangerous_obstacle(void);
	void set_dengerous_time_range(void);
	bool check_collision(void);
	bool is_collision(cv::Point2f& xr,cv::Point2f& xo,double& objs_r);


	bool change_selected_vel(void);
	void set_selected_vel(void);
	void set_default_vel_num(void);

	bool select_safety_vel(void);
	bool set_dangerous_angle(void);
	void clear_not_select_angle(void);
	void set_safety_vel(void);
	void set_stop_vel(void);

	void publish_velocity(void);
	void init_data(void);

	void publish_grid_map(void);

	void draw_dangerous_line(void);
	void show_cross_cloud(void);
	void debug_cloud(void);
	//iecon poster
	void subscribe_pcl(void);
	void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void conv_pcl_img(void);
};

#endif
