#include<vfh+.h>
#define MAX_EV 10000;
void vfh::set_polor_histogram(void){
	int W=map_wi;
	int H=map_hi;
	//init polor
	for(int i=0;i<ph.size();i++){
		ph[i]=-1;
	}
	//trans grid to polor
	for(int h=0;h<H;h++){
		uint8_t *pg = grid_map.ptr<uint8_t>(h);
		for(int w=0;w<W;w++){
			if(pg[w]>0){
				float d,th;
				trans_point_grid_to_polor(w,h,d,th);
				if(th<min_range||th>max_range){
					continue;
				}
				//障害物とロボットの大きさを考慮する必要がある(未実装)
				//trans th(float) to thi(int)
				int thi=(int)(th-max_range);
				if(ph[thi]<0 || ph[thi]>d){
					ph[thi]=d;
				}
			}
		}
	}
}

float vfh::select_angle(void){

	int select_angle=min_range+(max_range-min_range)/2;//center angle
	float min_ev=MAX_EV;//evaluation value
	float block_d=1*1;//block distance^2
	float w1=1;
	float w2=0.5;
	float w3=1/(map_hf-cy);
	//select angle
	for(int i=0;i<ph.size();i++){
		if(block_d>ph[i]){
			continue;
		}
		float ev=+w1*std::abs(min_range+i*reso_range - std::atan(-(xgf.x-xrf.x)/(xgf.y-xrf.y)) )//論文第1項
			+w2*std::abs(th_t-ph[i]*reso_range)//論文第2項
			-w3*ph[i];//追加した評価式
		if(min_ev>ev){
			min_ev=ev;
			select_angle=min_range+i;
		}
	}
	return (float)(select_angle*reso_range);
}
