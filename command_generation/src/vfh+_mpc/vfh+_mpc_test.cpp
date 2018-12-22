#include<vfh+_mpc.h>
void vfh_class_test(VFH_MPC& vfh);

bool setting_RobotTestCondition(VFH_MPC& vfh_mpc,float reso);
bool setting_RobotExpCondition(VFH_MPC& vfh_mpc,float reso);
void setting_testcondition(VFH_MPC& vfh_mpc,float reso);
void setting_condition(VFH_MPC& vfh_mpc,float reso,int num);
void condition1(VFH_MPC& vfh_mpc,float reso);
void condition2(VFH_MPC& vfh_mpc,float reso);
void condition3(VFH_MPC& vfh_mpc,float reso);
void condition4(VFH_MPC& vfh_mpc,float reso);
void condition5(VFH_MPC& vfh_mpc,float reso);

bool vfh_test=false;

int main(int argc,char **argv)
{
	ros::init(argc,argv,"vfh_mpc_test");
	//vfh
	float W=10;
	float H=10;
	float reso=0.1;
	VFH_MPC vfh_mpc(W,H,reso);//10,10,0.1);
	
	//vfh_class_test
	if(vfh_test){
		vfh_class_test(vfh_mpc);
		return 0;
	}
	bool TEST_CONDITION=false;

	//vfh_mpc_class_test
    //read launch param
	ros::NodeHandle node_private("~");
	int initNum=1;
	if(node_private.getParam("simu_num",initNum)){
		ROS_INFO_STREAM("read num"<<initNum);
	}
	else{
		ROS_ERROR_STREAM("Failed to get avoidance_type_flag at:"<<ros::this_node::getName());
	}
	if(initNum==0){
		setting_RobotTestCondition(vfh_mpc,reso);
		setting_testcondition(vfh_mpc,reso);

	}
	else{
		setting_RobotExpCondition(vfh_mpc,reso);
		setting_condition(vfh_mpc,reso,initNum);

	}
	vfh_mpc.end_set_mv_obstacle_data();
	//--
	vfh_mpc.draw_mpc_path_mat();
	ROS_INFO("Done...\n");
	return 0;
}
bool setting_RobotTestCondition(VFH_MPC& vfh_mpc,float reso)
{
	//setting
	std::cout<<"wait...\n";
	//center point
	cv::Point2f cpt=cv::Point2f(0.0,0.0);
	//goal point
	cv::Point2f goal_pt=cv::Point2f(7.5-5,7.5-5);
	
	//set_param
	std::cout<<"set_param...\n";
	vfh_mpc.set_center_point(cpt.x,cpt.y);
	vfh_mpc.set_goal(goal_pt);
	//--set_robot_param(float x,float y, float r,float vt0,float th_t0)
	if(!vfh_mpc.set_robot_param(-2.5,-2.5,0.2,0.2,0.0))//-M_PI/2))
	{
		std::cout<<"Error: robot param\n";
		return false; 
	}
	//--set_command_limit(float dif_vel)
	vfh_mpc.set_command_limit(0.2);
	vfh_mpc.set_mov_time(0.05);
	return true;	
}
bool setting_RobotExpCondition(VFH_MPC& vfh_mpc,float reso)
{
	//setting
	std::cout<<"wait...\n";
	//center point
	cv::Point2f cpt=cv::Point2f(5.0,5.0);
	//goal point
	cv::Point2f goal_pt=cv::Point2f(5.0,10.0);
	
	//set_param
	std::cout<<"set_param...\n";
	vfh_mpc.set_center_point(cpt.x,cpt.y);
	vfh_mpc.set_goal(goal_pt);
	//--set_robot_param(float x,float y, float r,float vt0,float th_t0)
	if(!vfh_mpc.set_robot_param(5.0,0.1,0.2,0.2,M_PI/2))//)
	{
		std::cout<<"Error: robot param\n";
		return false; 
	}
	//--set_command_limit(float dif_vel)
	vfh_mpc.set_command_limit(0.1);
	vfh_mpc.set_mov_time(0.05);
	return true;	
}

void setting_testcondition(VFH_MPC& vfh_mpc,float reso){
	//grid_map
	ROS_INFO("grid_map...\n");
	bool floor=false;
	int obst_num=0;
	if(!floor){
		cv::Point2f obst_data=cv::Point2f(0.0,0.0);
		cv::Point2f obst_data2=cv::Point2f(2.0,1.8);
		cv::Point2f obst_data3=cv::Point2f(-1.0,1.0);
		cv::Point2f obst_data4=cv::Point2f(1.0,-0.5);
		cv::Point2f obst_data5=cv::Point2f(2.0,-1.0);
		cv::Point2f obst_data6=cv::Point2f(-2.0,1.0);
		cv::Point2f obst_data7=cv::Point2f(3.5,-2.0);
		//--set_static_obstacle_data(cv::Point2f& data)
		vfh_mpc.set_static_obstacle_data(obst_data);
		vfh_mpc.set_static_obstacle_data(obst_data2);
		vfh_mpc.set_static_obstacle_data(obst_data3);
		vfh_mpc.set_static_obstacle_data(obst_data4);
		vfh_mpc.set_static_obstacle_data(obst_data5);
		//vfh.set_static_obstacle_data(obst_data6);
		vfh_mpc.set_static_obstacle_data(obst_data7);
		obst_num=6;
	}
	else{
		for(int j=0;j<100;j++){
			for(int i=0;i<100;i++){
				int ti=20;
				//std::cout<<"i,j:"<<i<<","<<j<<"\n";
				if(i==ti||i==100-ti){	
					std::cout<<"i,j:"<<i<<","<<j<<"\n";
					cv::Point2f obst_data=cv::Point2f((float)i/10.0,(float)j/10.0);
					vfh_mpc.set_static_obstacle_data(obst_data);
					obst_num++;
				}
			}
		}
	}
	
	//----
	//set movin obstacle data
	ROS_INFO("set movin obstacle data...\n");
	if(!floor){
		//--def mvObst
		float wo=0.20;
		float ho=0.2;
		int obst_size=(int)(wo/reso*2);
		float vx=-0.0;
		float vy=-0.20;
		cv::Point2f x1=cv::Point2f(-2.0-wo,1.0-ho);
		std::vector<cv::Point2f> mvObst1;
		mvObst1.resize(673*376);
		int k=0;
		for(int i=0;i<obst_size;i++){
			for(int j=0;j<obst_size;j++){
				if(i==0||j==0||i==obst_size-1||j==obst_size-1){
					cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
					mvObst1[k++]=temp;
				}
			}
		}
		mvObst1.resize(k);
		//----obst2
		float wo2=0.2;
		float ho2=0.2;
		int obst_size2=(int)(wo2/reso*2);
		float vx2=-0.1;
		float vy2=0.0;
		cv::Point2f x2=cv::Point2f(1.0-wo2,-2.0-ho2);
		std::vector<cv::Point2f> mvObst2;
		mvObst2.resize(673*376);
		int k2=0;
		for(int i=0;i<obst_size2;i++){
			for(int j=0;j<obst_size2;j++){
				if(i==0||j==0||i==obst_size2-1||j==obst_size2-1){
					cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x2;
					mvObst2[k2++]=temp;
				}
			}
		}
		mvObst2.resize(k2);
		//--set mvObst
		ROS_INFO("mvObst...\n");
		vfh_mpc.set_mv_obstacle_data(mvObst1,vx,vy);
		vfh_mpc.set_mv_obstacle_data(mvObst2,vx2,vy2);
	}
	else{
		
		//--def mvObst
		float ro=0.2;
		float wo=ro/std::sqrt(2);
		float ho=ro/std::sqrt(2);
		int obst_size=(int)(wo/reso*2);
		float vx=-0.0;
		float vy=-0.20;
		cv::Point2f x1=cv::Point2f(5.4-wo,10.0-ho);
		std::vector<cv::Point2f> mvObst1;
		mvObst1.resize(673*376);
		int k=0;
		for(int i=0;i<obst_size;i++){
			for(int j=0;j<obst_size;j++){
				if(i==0||j==0||i==obst_size-1||j==obst_size-1){
					cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
					mvObst1[k++]=temp;
				}
			}
		}
		mvObst1.resize(k);
		vfh_mpc.set_mv_obstacle_data(mvObst1,vx,vy);
		
		std::vector<cv::Point2f> mvObst2;
		k=0;
		x1=cv::Point2f(4.6-wo,8.0-ho);		
		mvObst2.resize(673*376);
		for(int i=0;i<obst_size;i++){
			for(int j=0;j<obst_size;j++){
				if(i==0||j==0||i==obst_size-1||j==obst_size-1){
					cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
					mvObst2[k++]=temp;
				}
			}
		}
		mvObst2.resize(k);
		vfh_mpc.set_mv_obstacle_data(mvObst2,vx,vy);
				
	}	
	
}
//experiment condition
void setting_condition(VFH_MPC& vfh_mpc,float reso,int num){
	ROS_INFO("chosing condition...\n");	
	switch(num){
		case 1: 
			condition1(vfh_mpc,reso);
			break;
		case 2:
			condition2(vfh_mpc,reso);
			break;
		case 3:
			condition3(vfh_mpc,reso);
			break;
		case 4:
			condition4(vfh_mpc,reso);
			break;
		case 5:
			condition5(vfh_mpc,reso);
			break;
	}
}
void condition1(VFH_MPC& vfh_mpc,float reso){
	ROS_INFO("seting condition1...\n");
	//--def mvObst
	float ro=0.2;
	float wo=ro/std::sqrt(2);
	float ho=ro/std::sqrt(2);
	int obst_size=(int)(wo/reso*2);
	float v=0.2;
	v=0.3;
	float th=M_PI;
	float vx=v*cos(th);
	float vy=v*sin(th);
	//std::cout<<"vx,vy:"<<vx<<","<<vy<<"\n";
	cv::Point2f x1=cv::Point2f(10-wo,5-ho);
	//std::cout<<"x1:"<<x1<<"\n";
	std::vector<cv::Point2f> mvObst1;
	mvObst1.resize(673*376);
	int k=0;
	for(int i=0;i<obst_size;i++){
		for(int j=0;j<obst_size;j++){
			if(i==0||j==0||i==obst_size-1||j==obst_size-1){
				cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
				mvObst1[k++]=temp;
			}
		}
	}
	mvObst1.resize(k);
	vfh_mpc.set_mv_obstacle_data(mvObst1,vx,vy);
}
void condition2(VFH_MPC& vfh_mpc,float reso){
	ROS_INFO("seting condition2...\n");
	//--def mvObst
	float ro=0.2;
	float wo=ro/std::sqrt(2);
	float ho=ro/std::sqrt(2);
	int obst_size=(int)(wo/reso*2);
	float v=0.2;
	float th=-M_PI*3.0/4.0;
	float vx=v*cos(th);
	float vy=v*sin(th);
	cv::Point2f x1=cv::Point2f(5.0+5/std::sqrt(2)-wo,5.0+5/std::sqrt(2)-ho);
	std::vector<cv::Point2f> mvObst1;
	mvObst1.resize(673*376);
	int k=0;
	for(int i=0;i<obst_size;i++){
		for(int j=0;j<obst_size;j++){
			if(i==0||j==0||i==obst_size-1||j==obst_size-1){
				cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
				mvObst1[k++]=temp;
			}
		}
	}
	mvObst1.resize(k);
	vfh_mpc.set_mv_obstacle_data(mvObst1,vx,vy);
}
void condition3(VFH_MPC& vfh_mpc,float reso){
	ROS_INFO("seting condition3...\n");
	//--def mvObst
	float ro=0.2;
	float wo=ro/std::sqrt(2);
	float ho=ro/std::sqrt(2);
	int obst_size=(int)(wo/reso*2);
	float v=0.2;
	float th=-M_PI/2;
	float vx=v*cos(th);
	float vy=v*sin(th);
	cv::Point2f x1=cv::Point2f(5.0-wo,10.0-ho);//5.0-wo,10.0-ho);
	std::vector<cv::Point2f> mvObst1;
	mvObst1.resize(673*376);
	int k=0;
	for(int i=0;i<obst_size;i++){
		for(int j=0;j<obst_size;j++){
			if(i==0||j==0||i==obst_size-1||j==obst_size-1){
				cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
				mvObst1[k++]=temp;
			}
		}
	}
	mvObst1.resize(k);
	vfh_mpc.set_mv_obstacle_data(mvObst1,vx,vy);
}
void condition4(VFH_MPC& vfh_mpc,float reso){
	ROS_INFO("seting condition4...\n");
	//--def mvObst
	float ro=0.2;
	float wo=ro/std::sqrt(2);
	float ho=ro/std::sqrt(2);
	int obst_size=(int)(wo/reso*2);
	float v=0.2;
	float th=-M_PI/4.0;
	float vx=v*cos(th);
	float vy=v*sin(th);
	cv::Point2f x1=cv::Point2f(5.0-5/std::sqrt(2)-wo,5.0+5/std::sqrt(2)-ho);
	std::vector<cv::Point2f> mvObst1;
	mvObst1.resize(673*376);
	int k=0;
	for(int i=0;i<obst_size;i++){
		for(int j=0;j<obst_size;j++){
			if(i==0||j==0||i==obst_size-1||j==obst_size-1){
				cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
				mvObst1[k++]=temp;
			}
		}
	}
	mvObst1.resize(k);
	vfh_mpc.set_mv_obstacle_data(mvObst1,vx,vy);
}
void condition5(VFH_MPC& vfh_mpc,float reso){
	ROS_INFO("seting condition5...\n");
	//--def mvObst
	float ro=0.2;
	float wo=ro/std::sqrt(2);
	float ho=ro/std::sqrt(2);
	int obst_size=(int)(wo/reso*2);
	float v=0.2;
	float th=0;
	float vx=v*cos(th);
	float vy=v*sin(th);
	cv::Point2f x1=cv::Point2f(0.0-wo,5.0-ho);
	std::vector<cv::Point2f> mvObst1;
	mvObst1.resize(673*376);
	int k=0;
	for(int i=0;i<obst_size;i++){
		for(int j=0;j<obst_size;j++){
			if(i==0||j==0||i==obst_size-1||j==obst_size-1){
				cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
				mvObst1[k++]=temp;
			}
		}
	}
	mvObst1.resize(k);
	vfh_mpc.set_mv_obstacle_data(mvObst1,vx,vy);
}
//vfh class test sample func
//in vfh_test.cpp
void vfh_class_test(VFH_MPC& vfh)
{
	std::cout<<"wait...\n";
	//center point
	cv::Point2f cpt=cv::Point(0.0,0.0);
	//goal point
	cv::Point2f goal_pt=cv::Point2f(7.5-5,7.5-5);
	
	//set_param
	std::cout<<"set_param...\n";
	vfh.set_center_point(cpt.x,cpt.y);
	vfh.set_goal(goal_pt);
	//--set_robot_param(float x,float y, float r,float vt0,float th_t0)
	if(!vfh.set_robot_param(-2.5,-2.5,0.25,0.4,0.0))//-M_PI/2))
	{
		std::cout<<"Error: robot param\n";
		return ; 
	}
	//--set_command_limit(float dif_vel)
	vfh.set_command_limit(0.2);
	
	vfh.set_mov_time(0.05);
	//grid_map
	ROS_INFO("grid_map...\n");
	
	cv::Point2f obst_data=cv::Point2f(0.0,0.0);
	cv::Point2f obst_data2=cv::Point2f(2.0,1.8);
	cv::Point2f obst_data3=cv::Point2f(-1.0,1.0);
	cv::Point2f obst_data4=cv::Point2f(1.0,-0.5);
	cv::Point2f obst_data5=cv::Point2f(2.0,-1.0);
	cv::Point2f obst_data6=cv::Point2f(-2.0,1.0);
	cv::Point2f obst_data7=cv::Point2f(3.5,-2.0);
	int obst_num=672*376;
	//--set_obstacle_data(cv::Point2f& data)
	vfh.set_obstacle_data(obst_data);
	vfh.set_obstacle_data(obst_data2);
	vfh.set_obstacle_data(obst_data3);
	vfh.set_obstacle_data(obst_data4);
	vfh.set_obstacle_data(obst_data5);
	//vfh.set_obstacle_data(obst_data6);
	vfh.set_obstacle_data(obst_data7);
	
	//return ;
	//path_planning
	ROS_INFO("path_planning...\n");
	 vfh.draw_path_mat();
	std::cout<<"Done\n";
}

