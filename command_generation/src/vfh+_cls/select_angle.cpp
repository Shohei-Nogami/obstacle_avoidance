#include<vfh+.h>
#define MAX_EV 10000;
void vfh::set_polar_histogram(void){
	int W=map_wi;
	int H=map_hi;
	float max_polar=std::sqrt(map_hf*map_hf+map_wf*map_wf);
	//init polor
	for(int i=0;i<ph.size();i++){
		ph[i]=max_polar;
	}
	//std::cout<<"ph.size():"<<ph.size()<<"\n";
	// std::cout<<"min,th,max:"<<th_t*180/M_PI-(max_range-min_range)/2
	// 	<<","<<th_t*180/M_PI<<","<<th_t*180/M_PI+(max_range-min_range)/2<<"\n";
	//conv grid to polor
	for(int h=0;h<H;h++){
		uint8_t *pg = grid_map.ptr<uint8_t>(h);
		for(int w=0;w<W;w++){
			if(pg[w]>0){
				float d,th;
				trans_point_grid_to_polor(w,h,d,th);
				//std::cout<<"w,h,d,th:"<<w<<","<<h<<","<<d<<","<<th<<"\n";
				//th=(th+th_t)*180/M_PI;
				//th=(th-th_t)*180/M_PI;
				th=th*180/M_PI;
				//std::cout<<"th_t:"<<th_t*180/M_PI<<"\n";
				//std::cout<<th_t*180/M_PI-(max_range-min_range)/2<<"<"<<th<<"<"<<th_t*180/M_PI+(max_range-min_range)/2<<"\n";
				if(th<th_t*180/M_PI-(max_range-min_range)/2
					||th>th_t*180/M_PI+(max_range-min_range)/2){
					continue;
				}
				//conv th(float) to thi(int)
				int thi=(int)(th-(th_t*180/M_PI-(max_range-min_range)/2) );
				//std::cout<<"th,thi,d:"<<th<<","<<thi<<","<<d<<"\n";
				//float dth=std::atan2(d,cr+rr)*180/M_PI;
				float margin_r=0.2;
				float dth=std::atan2(cr+rr+margin_r,d)*180/M_PI;
				int dthi=(int)dth+1;
				//std::cout<<"dth:"<<dth<<"\n";
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

float vfh::select_angle(void){

	int select_angle=min_range+(max_range-min_range)/2;//center angle
	float min_ev=MAX_EV;//evaluation value
	float block_d=1.5*1.5;//block distance^2
	float w1=1;
	float w2=0.5;
	float w3=1/(map_hf-cy);
	//select angle
	for(int i=0;i<ph.size();i++){
		//std::cout<<"ph["<<i<<"]:"<<ph[i]<<"\n";
		if(block_d>ph[i]){
			continue;
		}
		float th_p=std::atan2((xgf.y-xrf.y),(xgf.x-xrf.x))*180/M_PI;
		float th_s= th_t*180/M_PI+i*reso_range-(max_range-min_range)/2;
		float dif_th=th_p-th_s;
		if(std::abs(dif_th)>180){
			if(dif_th<0){
				dif_th+=360;
			}
			else{
				dif_th-=360;
			}
		}
		float ev=+w1*std::abs(dif_th)//論文第1項
			+w2*std::abs(i*reso_range-(max_range-min_range)/2)//論文第2項
			-w3*ph[i]//追加した評価式
			;

		if(min_ev>ev){
			min_ev=ev;
			select_angle=i;
		}
	}
	return (float)(select_angle*reso_range-(max_range-min_range)/reso_range/2)*M_PI/180+th_t;
}
