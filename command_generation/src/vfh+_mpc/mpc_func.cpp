#include<vfh+_mpc.h>

float& VFH_MPC::get_pot_xt(const cv::Point2i& xti) {
	return pot_mapt.at<float>(xti.y, xti.x);
}
void VFH_MPC::set_polar_histogram(cv::Mat& grid_map_temp,cv::Point2f& xrft,float& th_tt){
	int W=map_wi;
	int H=map_hi;
	float max_polar=std::sqrt(map_hf*map_hf+map_wf*map_wf);
	//init polor
	for(int i=0;i<ph.size();i++){
		ph[i]=max_polar;
	}
	//conv grid to polor
	for(int h=0;h<H;h++){
		uint8_t *pg = grid_map_temp.ptr<uint8_t>(h);
		for(int w=0;w<W;w++){
			if(pg[w]>0){
				float d,th;
				trans_point_grid_to_polor(w,h,xrft,d,th);
				th=th*180/M_PI;
				if(th<th_tt*180/M_PI-(max_range-min_range)/2
					||th>th_tt*180/M_PI+(max_range-min_range)/2){
					continue;
				}
				//conv th(float) to thi(int)
				int thi=(int)(th-(th_tt*180/M_PI-(max_range-min_range)/2) );
				float margin_r=0.2;
				float dth=std::atan2(cr+rr+margin_r,d)*180/M_PI;
				int dthi=(int)dth+1;
				
				for(int k=0;k<dthi*2;k++){
					int block_th=thi+k-dth;
					if(block_th<0||block_th>=(int)ph.size()){
						continue;
					}
					if(ph[block_th]<0 || ph[block_th]>d){
						ph[block_th]=d;
					}
				}
			}
		}
	}
}

float VFH_MPC::select_angle(const cv::Point2f& xrft,float& cost,float& th_t0) {
	float MAX_EV=10000;
	int select_angle = min_range + (max_range - min_range) / 2;//center angle
	float min_ev = MAX_EV;//evaluation value
	float block_d = 1.5*1.5;//block distance^2
	float w1 = 1;
	float w2 = 0.5;
	float w3 = 1 / (map_hf - cy);
	//select angle
	for (int i = 0; i < ph.size(); i++) {
		//std::cout << "ph[" << i << "]:" << ph[i] << "\n";
		if (block_d > ph[i]) {
			continue;
		}
		float th_p = std::atan2((xgf.y - xrft.y), (xgf.x - xrft.x)) * 180 / M_PI;
		float th_s = th_t0 * 180 / M_PI + i * reso_range - (max_range - min_range) / 2;
		float dif_th = th_p - th_s;
		if (std::abs(dif_th) > 180) {
			if (dif_th < 0) {
				dif_th += 360;
			}
			else {
				dif_th -= 360;
			}
		}
		float ev=+w1*std::abs(dif_th)//論文第1項
			+w2*std::abs(i*reso_range-(max_range-min_range)/2)//論文第2項
			-w3*ph[i]//追加した評価式
			;

		if (min_ev > ev) {
			min_ev = ev;
			select_angle = i;
		}
	}
	cost = min_ev;
	return (float)(select_angle*reso_range - (max_range - min_range) / reso_range / 2)*M_PI / 180 + th_t0;
}


//retunr sum cost
double VFH_MPC::culc_cost(cv::Point2f& xrft0, const float v0, const float& time_range)
{
	//debug 用
	ros::NodeHandle n;
	ros::Rate rate(1000);
	//set data
	double sum_cost=0;
	float tr=time_range;
	cv::Point2f xrft=xrft0;
	cv::Point2i xrit;
	float th_t0=th_t;
	//init mv data
	clear_move_data();
	//障害物データの数
	int obst_num=(int)obst_pti.size()+mv_data_size;
	//ゴール時の重み
	float vrate=2;
	//更新用グリッドマップ
	cv::Mat grid_map_t;

	while (ros::ok() && tr > 0)
	{
		//float to int
		trans_point_f_to_i(xrft, xrit);
		//std::cout<<"xrft,xgf:"<<xrft<<"-->"<<xgf<<"\n";
		//set grid map(t)
		grid_map_t = grid_map.clone();
		
		ROS_INFO("add_mv_pot...while\n");
		//add_mv_grid();
		add_mv_grid(grid_map_t);		
		//ゴールセルに到達したら終了
		if (xrit.x == xgi.x && xrit.y == xgi.y)
		{
			std::cout << "Goal\n";
			if (sum_cost > 0) {
				sum_cost /= vrate;
			}
			else {
				sum_cost *= vrate;
			}
			return -DBL_MAX * (tr / time_range);
		}
		if(check_collision(xrft)){//collision
			ROS_INFO("mpc collision");
			return DBL_MAX-(time_range-tr);
		}
		// ROS_INFO("set_grad...while\n");
		float w, v,angle;
		float cost = 0;
		//set_polar_histogram();
		set_polar_histogram(grid_map_t,xrft,th_t0);
		angle = select_angle(xrft,cost,th_t0);
		set_command_vel(th_t0,angle, v, w);
		std::cout<<"set_command_vel(select_angle(cost,th_t0), v, w);\n";
		//add cost
		sum_cost += cost;

		std::cout<<"float l = v * mv_t;\n";
		float l = v * mv_t;
		th_t0 = th_t0 + w * mv_t;
		xrft.x = xrft.x + l * cos(th_t0);
		xrft.y = xrft.y + l * sin(th_t0);
		//��Q���̈ړ�
		ROS_INFO("move_obstacle_data...while\n");
		move_obstacle_data(mv_t);
		//debug
		// ROS_INFO("set_pub_mpc_debug_images()\n");
		set_pub_mpc_debug_images(xrit);	
		//rate.sleep();

		tr -= mv_t;
		ROS_INFO("end...while\n");
	}
	return sum_cost;
}

float VFH_MPC::get_speed(const cv::Point2f& xrft0, const float& vrt00)
{
	cv::Point2f xrft = xrft0;
	cv::Point2i xrit;
	float vrt0 = vrt;//
	float delta_v = 0.05;
	float vrt1 = vrt + delta_v;
	
	// std::cout<<"vrt:"<<vrt<<"\n";
	float max_v = 0.3;
	float min_v = 0.1;
	//
	double cost0 = 0;
	double cost1 = 0;
	float time_range = 5;
	//
	float opt_v = vrt;
	//
	//std::cout<<"vrt0:"<<vrt0<<"\n";
	if (vrt1 > max_v) {
		vrt1 = vrt0;//+delta_v;
		vrt0 = vrt0 - delta_v;
	}
	//Process Once
	//std::cout<<"xrft0,xrft:"<<xrft0<<","<<xrft<<"\n";
	ROS_INFO("culc_cost0...\n");
	cost0 = culc_cost(xrft, vrt0, time_range);
	ROS_INFO("culc_cost1...\n");
	cost1 = culc_cost(xrft, vrt1, time_range);
	//predict param
	int search_num = 10;
	float pot_th = 0.10;//10%
	float pot_rate;
	//gradient v
	float grad_v;
	//
	bool flag01 = false;
	bool flag10 = false;
	ROS_INFO("while...\n");
	while (search_num-- > 0 && ros::ok()) {
		grad_v = (cost1 - cost0) / delta_v;
		
		//�ۗ�
		//if(grad_v>0){//cost0<cost1
		//std::cout<<"vrt("<<vrt0<<","<<vrt1<<")\n";
		//std::cout<<"cost("<<cost0<<","<<cost1<<")\n";
		// ROS_INFO("cost0,cost1:(%f,%f)\n",cost0,cost1);
		std::cout << "opt_v(" << opt_v << ")\n";
		if (cost0 <= cost1) {
			if (vrt0 >= max_v) {
				return max_v;
			}
			if (vrt0 <= min_v) {
				return min_v;
			}
			//ROS_INFO("vrt0<vrt1:(%f,%f)\n",vrt0,vrt1);
			opt_v = vrt0;
			vrt1 = vrt0;
			cost1 = cost0;
			vrt0 = vrt0 - delta_v;
			cost0 = culc_cost(xrft, vrt0, time_range);
			flag01 = true;
		}
		else {//cost1<cost0:vrt1<vrt0
			if (vrt1 >= max_v) {
				return max_v;
			}
			if (vrt1 <= min_v) {
				return min_v;
			}
			//ROS_INFO("vrt0>vrt1:(%f,%f)\n",vrt0,vrt1);
			opt_v = vrt1;
			vrt0 = vrt1;
			cost0 = cost1;
			vrt1 = vrt0 + delta_v;
			cost1 = culc_cost(xrft, vrt1, time_range);
			flag10 = true;
		}
		if (flag01&&flag10)
			break;

	}
	return opt_v;
}


