#include<vfh+_mpc.h>

/*
//obstacle position:pt,robot position:xti
float VFH_MPC::culc_mv_obstacle_fr(const cv::Point2i xti, const int& obstNum) {
	float sum_mvfr = 0;
	bool break_flag = false;
	for (int n = 0; n < mv_obsts.size(); n++) {
		cv::Point2i pti;
		//cv::Point2f *ptn=mv_obsts[n].data;
		for (int k = 0; k < mv_obsts[n].data.size(); k++)
		{
			cv::Point2f pt = mv_obsts[n].data[k];
			//cv::Point2f pt=ptn[k];
			pt.x += mv_obsts[n].mvx;
			pt.y += mv_obsts[n].mvy;
			pt.x += mv_obsts[n].mvxt;
			pt.y += mv_obsts[n].mvyt;
			if (trans_point(pt, pti))
			{
				float dis = culc_dis(xti.x, xti.y, pti.x, pti.y);
				sum_mvfr += culc_fr(dis, obstNum);
				if (dis < (rr + cr)) {
					break_flag = true;
					break;
				}
			}
			else {
				sum_mvfr += 0;
			}
		}
		if (break_flag) {
			break;
		}
	}
	if (mv_data_size > 0)
		sum_mvfr /= obstNum;
	return sum_mvfr;

}
*/
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
			pt.y += mv_obsts[n].mvyt;
			if (trans_point(pt, pti))
			{
				int ch_g = grid_mapt.channels();
				uint8_t *pgt = grid_mapt.ptr<uint8_t>(pti.y);
				if (pgt[pti.x * ch_g] == 0) {
					if (pgt[pti.x * ch_g] < 255) {
						pgt[pti.x * ch_g]++;
					}
				}
			}
		}
	}
}
