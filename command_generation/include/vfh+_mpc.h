#ifndef INCLUDE_VFH_MPC_CLASS
#define INCLUDE_VFH_MPC_CLASS

#include<vfh+.h>
#include<fstream>
#include<command_generation/robot_odm.h>
#include<nav_msgs/Odometry.h>

class VFH_MPC : public vfh
{
private:
	float pot_x0, pot_x1;
	float pot_y0, pot_y1;
	float grad_xt, grad_yt;
	float pot_xt0;
	cv::Point2f xrf0;
	cv::Mat pot_mapt;
	cv::Mat grid_mapt;
	float time_range;
	struct mv_obst {
		float vx;
		float vy;
		float mvx;
		float mvy;
		//debug
		float mvxt;
		float mvyt;
		//
		std::vector<cv::Point2f> data;
	};
	std::vector<mv_obst> mv_obsts;
	int mv_obsts_size;
	int mv_data_size;
	//debug
	cv::Mat mpc_debug_image;
	//cv::Mat pot_maptt;
	std::vector<cv::Point2i> obst_pti;
public:
	//vfh+_mpc.cpp
	VFH_MPC(float width, float height, float resolution);
	~VFH_MPC();
	//set_data.cpp
	void set_static_obstacle_data(const cv::Point2f& data);
	void set_mv_obstacle_data(const std::vector<cv::Point2f>& pts, const float vx0, const float vy0);
	void end_set_mv_obstacle_data(void);
	void clear_mv_obstacle_data(void);
	void move_obstacle_data(float& time);
	void clear_move_data(void);
	int get_obst_num(void);
	cv::Mat& get_grid_map(void);
	//grid_map.cpp in apf class sources
	////void set_command_vel(cv::Point2i& xri0,float& w);
	//void set_command_vel(const cv::Point2i& xri0, const float& v0, float& v, float& w, float& th_t0);
	//mv_pot_map.cpp
	//float culc_mv_obstacle_fr(const cv::Point2i xti, const int& obstNum);
	//void add_mv_pot(const cv::Point2i xti, const int& obstNum);
	void add_mv_grid(void);
	void add_mv_grid(cv::Mat& grid_map_temp);
	//mpc_func.cpp
	float& get_pot_xt(const cv::Point2i& xti);
	void set_polar_histogram(cv::Mat& grid_map_temp);
	// bool set_grad(const cv::Point2i& xti);
	double culc_cost(cv::Point2f& xrft0, const float v0, const float& time_range);
	float get_speed(const cv::Point2f& xrft0, const float& vrt00);
	float select_angle(float& cost,float& th_t0);
	//debug.cpp
	void set_pub_mpc_debug_images(const cv::Point2i& xrit0);
	void draw_mv_obst(void);
	// void draw_mpc_path_mat(void);
	bool check_collision(const cv::Point2f xrf00);
	void past_time(const float& time);
	// bool set_grad0(const cv::Point2i& xti);
};
#endif
