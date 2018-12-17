#include<vfh+_mpc.h>


void VFH_MPC::add_mv_grid(void) {
	
	for (int n = 0; n < mv_obsts.size(); n++) {
		cv::Point2i pti;
		for (int k = 0; k < mv_obsts[n].data.size(); k++)
		{
			cv::Point2f pt = mv_obsts[n].data[k];
			pt.x += mv_obsts[n].mvx;
			pt.y += mv_obsts[n].mvy;
			pt.x += mv_obsts[n].mvxt;
			pt.y += mv_obsts[n].mvyt;
			if (trans_point(pt, pti))
			{
				uint8_t *pgt = grid_mapt.ptr<uint8_t>(pti.y);
				if (pgt[pti.x ] < 255) {
					pgt[pti.x ]=255;
				}
			}
		}
	}
}
void VFH_MPC::add_mv_grid(cv::Mat& grid_map_temp) {
	
	for (int n = 0; n < mv_obsts.size(); n++) {
		cv::Point2i pti;
		for (int k = 0; k < mv_obsts[n].data.size(); k++)
		{
			cv::Point2f pt = mv_obsts[n].data[k];
			pt.x += mv_obsts[n].mvx;
			pt.y += mv_obsts[n].mvy;
			pt.x += mv_obsts[n].mvxt;
			pt.y += mv_obsts[n].mvyt;
			if (trans_point(pt, pti))
			{
				uint8_t *pgt = grid_map_temp.ptr<uint8_t>(pti.y);
				if (pgt[pti.x] < 255) {
					pgt[pti.x]=255;
				}
			}
		}
	}
}
