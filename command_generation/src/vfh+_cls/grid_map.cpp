#include<vfh+.h>


bool vfh::check_gridmap_format(cv::Mat& map){
	
	//check map size
	if(map.cols!=map_wi || map.rows!=map_hi)
		return false;
		
	//check map format
	if(map.depth()!=CV_8U || map.channels()!=1)
		return false;
		 
	return true;
}

void vfh::set_grid_map(cv::Mat& map){
	grid_map=map.clone();
}

void vfh::set_obstacle_data(const cv::Point2f& data)
{
	cv::Point2i data_gp;
	std::cout<<"data:"<<data<<"\n";
	// if(trans_point(data,data_gp))
	if(trans_point_f_to_i(data,data_gp))
	{
		// int ch_g = grid_map.channels();
		uint8_t *pgrid = grid_map.ptr<uint8_t>(data_gp.y);
		std::cout<<"data_gp.y:"<<data_gp.y<<"\n";
		std::cout<<"pgrid["<<data_gp.x<<"]:"<<(int)pgrid[data_gp.x]<<"\n";
		if(pgrid[data_gp.x]<255){
			pgrid[data_gp.x]++;
		}
		std::cout<<"pgrid["<<data_gp.x<<"]:"<<(int)pgrid[data_gp.x]<<"\n";

	}
	else
	{
		// std::cout<<"ObstPoint is not in grid map\n";
	}}

void vfh::clear_grid_map(void)
{
	cv::Mat m_temp = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC1);
	grid_map=m_temp.clone(); 
}


