#include<vfh+.h>

void vfh::set_command_vel(float angle,float& v,float& w)
{
        float delta_th=angle-th_t;
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
			
}