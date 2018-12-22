#include<vfh+.h>

float& vfh::get_reso(void){
	return reso;
}


cv::Point2f& vfh::get_posf(void){
	return xrf;
}
cv::Point2i& vfh::get_posi(void){
	return xri;
}
cv::Point2f& vfh::get_goal_posf(void){
	return xgf;
}
cv::Point2i& vfh::get_goal_posi(void){
	return xgi;
}
float& vfh::get_ori(void){
	return th_t;
}
float& vfh::get_vel(void){
	return vrt;
}
float& vfh::get_ang_vel(void){
	return wrt;
}

void vfh::set_grid_param(float width,float height,float resolution){
	//マップデータの設定と初期化
	map_wf=width;
	map_hf=height;
	reso=resolution;
	map_wi=(int)(map_wf/resolution);
	map_hi=(int)(map_hf/resolution);
	cr=std::sqrt(2)*reso/2;
	
	
	//cv::Mat
	//int
	cv::Mat m_temp = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC1);
	grid_map=m_temp.clone();
	
	//debug
	debug_image = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC3);
}
//グリッドマップの中心座標（クローバル座標）
void vfh::set_center_point(float cpx,float cpy){
	cx=cpx;
	cy=cpy;
}

void vfh::set_goal(cv::Point2f& goal_2f)
{
	if(trans_point(goal_2f,xgi,xgf))
	{
		
	}
	else
	{
		std::cout<<"GoalPoint is not in grid map\n";
	}
}
bool vfh::set_robot_param(float x,float y, float r,float vt0,float th_t0)
{
	xr.x=x;
	xr.y=y;
	rr=r;
	vrt=vt0;
	th_t=th_t0;//反時計回り(0~360)
	
	if(trans_point(xr,xri,xrf))
	{
		//std::cout<<"xr,xri,xrf:"<<xr<<","<<xri<<","<<xrf<<"\n";
		return true;
	}
	else{
		std::cout<<"RobotPoint is not in grid map\n";
		return false;
	}	
}
bool vfh::update_robot_param(const float& x,const float& y,const float& th_t0,const float& vt0,const float& wt0){
	xr.x=x;
	xr.y=y;
	vrt=vt0;
	wrt=wt0;
	th_t=th_t0;//反時計回り(0~360)
	
	if(trans_point(xr,xri,xrf))
	{
		//std::cout<<"xr,xri,xrf:"<<xr<<","<<xri<<","<<xrf<<"\n";
		return true;
	}
	else{
		std::cout<<"RobotPoint is not in grid map\n";
		return false;
	}	
}
void vfh::set_command_limit(float dif_vel)
{
	float d=0.138;
	max_w=dif_vel/(2*d);
}
void vfh::set_mov_time(float time)
{
	mv_t=time;
}
