#include<vfh+_mpc.h>

void VFH_MPC::set_pub_mpc_debug_images(const cv::Point2i& xrit0)
{
	
	int W=map_wi;
	int H=map_hi;
	mpc_debug_image = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC3);
	
	for(int h0=0;h0<H;h0++){
		uint8_t *pd = grid_map.ptr<uint8_t>(h0);
		cv::Vec3b *p3d = mpc_debug_image.ptr<cv::Vec3b>(h0);
		for(int w0=0;w0<W;w0++){
			uint8_t pot=pd[w0];
			if(pot>0)
			{
				// std::cout<<"colored \n";
				p3d[w0][2]=255;
			}
			else
			{

			}
		}
	}
	
	mpc_debug_image.at<cv::Vec3b>(xrit0.y, xrit0.x)[0] = 255;
	mpc_debug_image.at<cv::Vec3b>(xrit0.y, xrit0.x)[1] = 255;
	mpc_debug_image.at<cv::Vec3b>(xrit0.y, xrit0.x)[2] = 255;
	// std::cout<<"draw static obstacle\n";
	mpc_debug_image.at<cv::Vec3b>(xgi.y,xgi.x)[0] =255;
	mpc_debug_image.at<cv::Vec3b>(xgi.y,xgi.x)[1] =255;
	mpc_debug_image.at<cv::Vec3b>(xgi.y,xgi.x)[2] =0;
	draw_mv_obst();
	// std::cout<<"draw dynamic obstacle\n";

	publish_debug_image(mpc_debug_image);
	//ROS_INFO("published_debug_image\n");	
}
void VFH_MPC::draw_mv_obst(void) {
	//std::cout<<"void VFH_MPC::draw_mv_obst(void){\n";
	// std::cout<<"mv_obsts.size():"<<mv_obsts.size()<<"\n";
	for (int n = 0; n < mv_obsts.size(); n++) {
		cv::Point2i pti;
		//std::cout<<"mv_obsts[n].data.size():"<<mv_obsts[n].data.size()<<"\n";
		for (int k = 0; k < mv_obsts[n].data.size(); k++)
		{
			cv::Point2f pt = mv_obsts[n].data[k];
			//std::cout<<"pt0:"<<pt<<"\n";
			pt.x += mv_obsts[n].mvx;
			pt.y += mv_obsts[n].mvy;
			pt.x += mv_obsts[n].mvxt;
			pt.y += mv_obsts[n].mvyt;
			// std::cout<<"pt:"<<pt<<"\n";
			if (trans_point(pt, pti))
			{
				// std::cout<<"pti:"<<pti<<"\n";
				// std::cout<<"W,H:"<<mpc_debug_image.cols<<","<<mpc_debug_image.rows<<std::endl;
				// std::cout<<"W,H:"<<map_wi<<","<<map_hi<<std::endl;
				
				mpc_debug_image.at<cv::Vec3b>(pti.y, pti.x)[0] = 0;
				mpc_debug_image.at<cv::Vec3b>(pti.y, pti.x)[1] = 255;
				mpc_debug_image.at<cv::Vec3b>(pti.y, pti.x)[2] = 0;
			}
		}
	}
}

bool VFH_MPC::check_collision(const cv::Point2f xrf00) {

	for (int n = 0; n < mv_obsts.size(); n++) {
		cv::Point2i pti;
		//std::cout<<"mv_obsts[n].data.size():"<<mv_obsts[n].data.size()<<"\n";
		for (int k = 0; k < mv_obsts[n].data.size(); k++)
		{
			cv::Point2f pt = mv_obsts[n].data[k];
			// std::cout<<"pt0:"<<pt<<"\n";
			pt.x -= cx;
			pt.y -= cy;
			pt.x += mv_obsts[n].mvx;
			pt.y += mv_obsts[n].mvy;
			pt.x += mv_obsts[n].mvxt;
			pt.y += mv_obsts[n].mvyt;
			// std::cout<<"pt,xrf00:"<<pt<<","<<xrf00<<"\n";
			float dis = std::sqrt((xrf00.x - pt.x)*(xrf00.x - pt.x) + (xrf00.y - pt.y)*(xrf00.y - pt.y));
			if (dis <= rr + cr)
			{
				// std::cout<<"dis:"<<dis<<"\n";
				return true;
			}
		}
	}
	for (int k = 0; k < obst_pti.size(); k++)
	{
		float dis = std::sqrt((xrf00.x - obst_pti[k].x)*(xrf00.x - obst_pti[k].x) + (xrf00.y - obst_pti[k].y)*(xrf00.y - obst_pti[k].y));
		if (dis < rr + cr) {
			return true;
		}
	}
	return false;
}

void VFH_MPC::past_time(const float& time) {
	//std::cout<<"void VFH_MPC::past_time(const float& time){\n";
	clear_move_data();
	for (int n = 0; n < mv_obsts.size(); n++) {
		mv_obsts[n].mvxt += mv_obsts[n].vx*time;
		mv_obsts[n].mvyt += mv_obsts[n].vy*time;

	}
	//std::cout<<"void VFH_MPC::past_time(const float& time){\n";
}
void VFH_MPC::draw_mpc_path_mat(void)
{
	ros::NodeHandle n;
	ros::Rate rate(100);
	int obst_num=(int)obst_pti.size()+mv_data_size;
	clear_move_data();
	float v0=vrt;
	float goal_time=0;
	// std::ofstream ofss("./vel_angVel.csv",std::ios::app);
	// ofss<<"time"<<","<<"x"<<","<<"y"<<","<<"v"<<","<<"w"<<","<<"th_t"<<","<<std::endl;

	// ros::NodeHandle nh1,nh2;
	// ros::Publisher pub1,pub2;
	// command_generation::robot_odm robot_odm;
	// nav_msgs::Odometry obst_odm;
	// pub1=nh1.advertise<command_generation::robot_odm>("robot_odm",1);
	// pub2=nh2.advertise<nav_msgs::Odometry>("obstacle_odm",1);

	// obst_odm.pose.pose.position.x=3.5;
	// obst_odm.pose.pose.position.y=0;
	// obst_odm.pose.pose.position.z=0;
	cv::Mat grid_map_t;

	ROS_INFO("clear_move_data");
	clear_move_data();
	while(ros::ok()){
		// //debug
		// robot_odm.x=xrf.x;
		// robot_odm.y=xrf.y;
		// obst_odm.pose.pose.position.x+=mv_t*0.2;
		// pub1.publish(robot_odm);
		// pub2.publish(obst_odm);
		grid_map_t = grid_map.clone();
		//float to int
		trans_point_f_to_i(xrf,xri);
		std::cout<<"xrf,xgf:"<<xrf<<"-->"<<xgf<<"\n";
		//ゴールセルに到達したら終了
		if(xri.x==xgi.x && xri.y==xgi.y)
		{
			std::cout<<"Goal\n";
			break;
		}
		//MPC
		ROS_INFO("get_speed");
		v0 = get_speed(xrf,vrt);
		//v0=vrt;
		std::cout<<"v0:"<<v0<<"\n";
		ROS_INFO("clear_move_data");
		clear_move_data();
		add_mv_grid(grid_map_t); 
		ROS_INFO("check_collision");
		if(check_collision(xrf))//collision
		{
			ROS_INFO("collision...\n");
			break;
		}
		//ロボットの命令速度算出
		float w, v,angle;
		float cost=0;
		v=v0;
		ROS_INFO("set_command_vel...\n");
		set_polar_histogram(grid_map_t,xrf,th_t);
		angle = select_angle(xrf,cost,th_t);
		set_command_vel(th_t,angle, v, w);
		std::cout<<"v0,vrt,w,th_t:"<<v0<<","<<vrt<<","<<w<<","<<th_t<<"\n";
		
		// ofss<<goal_time<<","<<xrf.x+cx<<","<<xrf.y+cy<<","<<v<<","<<w<<","<<th_t<<","<<std::endl;
		
		//ロボットの移動
		//mv_t:移動時間
		float l=v*mv_t;
		th_t=th_t+w*mv_t;
		xrf.x=xrf.x + l*cos(th_t);
		xrf.y=xrf.y + l*sin(th_t);
		//速度変化
		//ロボットの速度は1時遅れ系で変化すると仮定
		//目標値v0,現在速度vrt
		float Tr=0.25/1000;//マブチ3Vモータを参考
		//vrt=vrt+(v0-vrt)*(1-exp(-Tr*dt));
		vrt=vrt+(v0-vrt)*(exp(-Tr*mv_t));
		std::cout<<"(1-exp(-Tr*dt)):"<<(1-exp(-Tr*mv_t))<<"\n";
		//debug
		//ROS_INFO("set_pub_mpc_debug_images...\n");
		past_time(mv_t);
		set_pub_mpc_debug_images(xri);
		//move obstacles
		rate.sleep();
		goal_time+=mv_t;	
	}
	//test
	// std::ofstream ofs("./goal_time.csv",std::ios::app);
	// ofs<<goal_time<<std::endl;
}