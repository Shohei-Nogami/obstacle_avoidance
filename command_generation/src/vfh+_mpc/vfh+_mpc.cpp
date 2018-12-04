#include<vfh_mpc.h>

VFH_MPC::VFH_MPC(float width, float height, float resolution)
	:mv_obsts_size(0), mv_data_size(0)
{
	std::cout << "APF_MPC\n";
	set_grid_param(width, height, resolution);
	mv_obsts.resize((int)(width / resolution * height / resolution));
	//debug
	mpc_debug_image = cv::Mat::zeros(cv::Size(map_hi, map_wi), CV_8UC3);
}

VFH_MPC::~VFH_MPC() {
	mpc_debug_image.release();
	pot_mapt.release();
}
