#include<vfh+.h>

void vfh::draw_path_mat(void)
{
	ros::NodeHandle n;
	ros::Rate rate(100);
	while(ros::ok()){
	
		//float to int
		trans_point_f_to_i(xrf,xri);
		std::cout<<"xrf,xgf:"<<xrf<<"-->"<<xgf<<"\n";
		std::cout<<"th:"<<std::atan2((xgf.y-xrf.y),(xgf.x-xrf.x))*180/M_PI<<"\n";
		//ゴールセルに到達したら終了
		if(xri.x==xgi.x && xri.y==xgi.y)
		{
			std::cout<<"Goal\n";
			break;
		}

		//ロボットの命令速度算出
		float w,v;
        set_polar_histogram();
		set_command_vel(select_angle(),v,w);
		// std::cout<<"v,w,th_t:"<<v<<","<<w<<","<<th_t<<"\n";
		//ロボットの移動
		//mv_t:移動時間
		float l=v*mv_t;
		th_t=th_t+w*mv_t;
		xrf.x=xrf.x + l*cos(th_t);
		xrf.y=xrf.y + l*sin(th_t);
		
		//debug
		set_pub_debug_images();
		rate.sleep();
				
	}
}

void vfh::set_pub_debug_images(void)
{
	int W=map_wi;
	int H=map_hi;
	int h;
//	#pragma omp parallel for
	for(h=0;h<H;h++){
		uint8_t *pg = grid_map.ptr<uint8_t>(h);
		cv::Vec3b *pd = debug_image.ptr<cv::Vec3b>(h);
		//#pragma omp parallel for
		for(int w=0;w<W;w++){
			if(pg[w]>0){
                pd[w][0]=0;
                pd[w][1]=255;
                pd[w][2]=0;
            }
		}
			
	}
	debug_image.at<cv::Vec3b>(xri.y,xri.x)[0] =255;
	debug_image.at<cv::Vec3b>(xri.y,xri.x)[1] =255;
	debug_image.at<cv::Vec3b>(xri.y,xri.x)[2] =255;
		
	debug_image.at<cv::Vec3b>(xgi.y,xgi.x)[0] =0;
	debug_image.at<cv::Vec3b>(xgi.y,xgi.x)[1] =0;
	debug_image.at<cv::Vec3b>(xgi.y,xgi.x)[2] =255;

	publish_debug_image(debug_image);
	
}
void vfh::publish_debug_image(const cv::Mat& temp_image){
	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=temp_image.clone();
	pub.publish(publish_cvimage->toImageMsg());

}

