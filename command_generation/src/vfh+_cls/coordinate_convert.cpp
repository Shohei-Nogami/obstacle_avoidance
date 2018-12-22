#include<vfh+.h>

//global point-> grid point
bool vfh::trans_point(const cv::Point2f& pt,cv::Point2i& pti){
	float map_ptx = map_wf/2 + (pt.x - cx);
	float map_pty = map_hf/2 + ( -(pt.y - cy) );
	
	// if(map_ptx<0 || map_ptx>map_wf)
	// 	return false;
		
	// if(map_pty<0 || map_pty>map_hf)
	// 	return false;
		
	pti.x =	(int)(map_ptx/reso);
	pti.y =	(int)(map_pty/reso);
	
	if(pti.x<0 || pti.x>=map_wi)
		return false;
		
	if(pti.y<0 || pti.y>=map_hi)
		return false;
	
	return true;
	
}
//global point-> grid point float and int
bool vfh::trans_point(const cv::Point2f& pt,cv::Point2i& pti,cv::Point2f& ptf){
	ptf.x = pt.x - cx;
	ptf.y = pt.y - cy;
	float map_ptx = map_wf/2 + ptf.x;
	float map_pty = map_hf/2 - ptf.y;
	
	
	// if(map_ptx<0 || map_ptx>map_wf)
	// 	return false;
		
	// if(map_pty<0 || map_pty>map_hf)
	// 	return false;

	pti.x = (int)(map_ptx/reso);
	pti.y = (int)(map_pty/reso);
	// std::cout<<"pti:"<<pti<<"\n";
	if(pti.x<0 || pti.x>=map_wi)
		return false;
		
	if(pti.y<0 || pti.y>=map_hi)
		return false;
	
	return true;
	
}
bool vfh::trans_point_f_to_i(const cv::Point2f& ptf,cv::Point2i& pti){

	float map_ptx = map_wf/2 + ptf.x;
	float map_pty = map_hf/2 - ptf.y;
	
	// if(map_ptx<0 || map_ptx>map_wf)
	// 	return false;
		
	// if(map_pty<0 || map_pty>map_hf)
	// 	return false;

	pti.x = (int)(map_ptx/reso);
	pti.y = (int)(map_pty/reso);
	if(pti.x<0 || pti.x>=map_wi)
		return false;
		
	if(pti.y<0 || pti.y>=map_hi)
		return false;

	return true;

}
void vfh::trans_point_grid_to_polor(const int xg,const int yg,float& d,float& th){
	float xg0 = xg*reso - map_wf/2;
	float yg0 = map_hf/2 - yg*reso;
	d=(xg0-xrf.x)*(xg0-xrf.x)+(yg0-xrf.y)*(yg0-xrf.y);
	th=std::atan2((yg0-xrf.y),(xg0-xrf.x));
	//std::cout<<"(xg0-xrf.x),(yg0-xrf.y):"<<(xg0-xrf.x)<<","<<(yg0-xrf.y)<<"\n";
	//std::cout<<"xr,xg,d,th:"<<xrf<<",("<<xg0<<","<<yg0<<"),"<<d<<","<<th<<"\n";
}
void vfh::trans_point_grid_to_polor(const int xg,const int yg,const cv::Point2f xrft,float& d,float& th){
	float xg0 = xg*reso - map_wf/2;
	float yg0 = map_hf/2 - yg*reso;
	d=(xg0-xrft.x)*(xg0-xrft.x)+(yg0-xrft.y)*(yg0-xrft.y);
	th=std::atan2((yg0-xrft.y),(xg0-xrft.x));
	//std::cout<<"(xg0-xrf.x),(yg0-xrf.y):"<<(xg0-xrf.x)<<","<<(yg0-xrf.y)<<"\n";
	//std::cout<<"xr,xg,d,th:"<<xrf<<",("<<xg0<<","<<yg0<<"),"<<d<<","<<th<<"\n";
}

