#include<vfh+.h>

void vfh::set_command_vel(float angle,float& v,float& w)
{
        float delta_th;

		//-180<th_t<180
		if(th_t>M_PI)
		{
			th_t-=2*M_PI;
		}
		else if(th_t<-M_PI)
		{
			th_t+=2*M_PI;
		}
		//偏差
		//angle 目標
		//th_t 現在地(フィードバックの値)
		delta_th= angle -th_t;

		//-180<th_t<180
		if(std::abs(delta_th)>M_PI){
			if(delta_th<0){
				delta_th+=2*M_PI;
			}
			else{
				delta_th-=2*M_PI;
			}
		}
		//角速度(P制御)
		float Kp=1;
		w=Kp*delta_th;
		if(w>max_w)
		{
			w=max_w;
		}
		else if(w<-max_w)
		{
			w=-max_w;
		}
		//速度可変
		//float d=0.138;
		//float dif_v = w*2*d;
		//v=vrt-std::abs(dif_v)/2;
		//速度は一定
		v=vrt;
		std::cout<<"v,w,th,th_t:"<<v<<","<<w<<","<<angle*180/M_PI<<","<<th_t*180/M_PI<<"\n";

}