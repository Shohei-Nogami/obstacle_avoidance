#include<command_generator.h>

bool PROCESS(command_generator& cgen, APF_MPC& apf_mpc);
bool PROCESS(command_generator& cgen, VFH_MPC& vfh_mpc);
void PROCESS_SIMULATION(command_generator& cgen, VFH_MPC& vfh_mpc);

float v0=0.2;//temp

int main(int argc,char **argv)
{
	ros::init(argc, argv, "command_genaration_mpc");
    command_generator cgen;
	//apf param
	float W=8;
	float H=8;
	float reso=0.15;
	//USED FUNCTION FLAG
	bool USE_APF_MPC = false;
	bool USE_VFH_MPC = true;
	bool USE_SIMU =true;

	//read launch param
	ros::NodeHandle node_private("~");
	bool initFlag;
	if(node_private.getParam("avoidance_type_flag",initFlag)){
		ROS_INFO_STREAM("read flag"<<initFlag);
		USE_APF_MPC = initFlag;
		USE_VFH_MPC = !initFlag;
	}
	else{
		ROS_ERROR_STREAM("Failed to get avoidance_type_flag at:"<<ros::this_node::getName());
	}
	//first odometry 
	std::cout << "waiting first odometry\n";
	// cgen.subscribe_odometry();
	
	APF_MPC apf_mpc(W, H, reso);//10,10,0.1);
	VFH_MPC vfh_mpc(W, H, reso);//10,10,0.1);

	if(USE_SIMU){
		PROCESS_SIMULATION(cgen,vfh_mpc);
		std::cout << "SIMULATION END \n";
		return 0;
	}
	
	//init each class param
	if (USE_APF_MPC) {
		if (!cgen.setting_RobotExpCondition(apf_mpc, reso)) {
			ROS_INFO("Setting failed!");
			return -1;
		}
	}
	else if (USE_VFH_MPC) {
		if (!cgen.setting_RobotExpCondition(vfh_mpc, reso)) {
			ROS_INFO("Setting failed!");
			return -1;
		}
	}
	//v init (vの初期値)
	float v_init=0;
	float w_init=0;
	cgen.update_RobotVel(v_init,w_init);
	//turtlebot setting
	geometry_msgs::Twist turtlebot_vel;
	std_msgs::Empty empty_msg;
	ros::NodeHandle nh,nh2;
	ros::Publisher pub,pub2;
	pub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);//速度msg
	pub2 = nh.advertise<std_msgs::Empty>("mobile_base/commands/reset_odometry", 1);//リセットオドメトリmsg

	turtlebot_vel.linear.x=0.2;
	turtlebot_vel.linear.y=0;
	turtlebot_vel.linear.z=0;
	turtlebot_vel.angular.x=0;
	turtlebot_vel.angular.y=0;
	turtlebot_vel.angular.z=0;
	
	//turtlebot自己位置リセットmsgのpublish
	int i=0;
	ros::Rate r=1;
	while(ros::ok()&&i++<2)
	{
		pub2.publish(empty_msg);
		r.sleep();
	}
	ROS_INFO("Process Starts!");

	//処理開始
	while(ros::ok()){
		//subscribe data
		cgen.subscribe_objects();
		cgen.subscribe_odometry();

		if (USE_APF_MPC) {
			if(!PROCESS(cgen, apf_mpc)){
				break;
			}
		}
		else if (USE_VFH_MPC){
			if(!PROCESS(cgen, vfh_mpc)){
				break;
			}
		}
	}
	return 0;
}
bool PROCESS(command_generator& cgen,APF_MPC& apf_mpc) {
	//update param and init data
	cgen.update_RobotPos(apf_mpc);
	apf_mpc.clear_grid_map();
	//recognize obstacle state(
	if (cgen.dicriminate_obstacle()) {
		cgen.set_obstacles(apf_mpc);
	}
	//create potential map
	ROS_INFO("create_pot_map");
	apf_mpc.create_pot_map();
	ROS_INFO("clear_move_data");
	apf_mpc.clear_move_data();
	//the position of robot convert global to grid(int)
	ROS_INFO("trans_point_f_to_i");
	apf_mpc.trans_point_f_to_i(apf_mpc.get_posf(), apf_mpc.get_posi());
	std::cout << "xrf,xgf:" << apf_mpc.get_posf() << "-->" << apf_mpc.get_goal_posf() << "\n";
	//ゴールセルに到達したら終了
	cv::Point xri = apf_mpc.get_posi();
	cv::Point xgi = apf_mpc.get_goal_posi();
	if (xri.x == xgi.x && xri.y == xgi.y)
	{
		std::cout << "Goal\n";
		return false;
	}
	//MPC
	ROS_INFO("get_speed");
	v0 = apf_mpc.get_speed(apf_mpc.get_posf(), apf_mpc.get_vel());
	ROS_INFO("clear_move_data");
	apf_mpc.clear_move_data();
	ROS_INFO("add_mv_pot");
	apf_mpc.add_mv_pot(apf_mpc.get_posi(), apf_mpc.get_obst_num());//int --> ref int
	ROS_INFO("set_grad");
	apf_mpc.set_grad(apf_mpc.get_posi());
	ROS_INFO("check_collision");
	if (apf_mpc.check_collision(apf_mpc.get_posf()))//collision
	{
		ROS_INFO("collision...\n");
		return false;
	}

	//ロボットの命令速度算出
	float w, v;
	ROS_INFO("set_command_vel...\n");
	apf_mpc.set_command_vel(apf_mpc.get_posi(), v0, v, w, apf_mpc.get_ori());
	//cgen.update_RobotVel(v,w);
	std::cout << "v,w:" << v << "," << w << "\n";
	cgen.publish_velocity(v, w);
	cgen.publish_wheel_velocity(v, w);
	apf_mpc.set_pub_mpc_debug_images(xri);
	return true;
}
bool PROCESS(command_generator& cgen, VFH_MPC& vfh_mpc) {

	//update param and init data
	cgen.update_RobotPos(vfh_mpc);
	vfh_mpc.clear_grid_map();
	//recognize obstacle state(
	if(cgen.dicriminate_obstacle()){
		cgen.set_obstacles(vfh_mpc);
	}
	ROS_INFO("clear_move_data");
	vfh_mpc.clear_move_data();
	//the position of robot convert global to grid(int)
	ROS_INFO("trans_point_f_to_i");
	vfh_mpc.trans_point_f_to_i(vfh_mpc.get_posf(), vfh_mpc.get_posi());
	std::cout << "xrf,xgf:" << vfh_mpc.get_posf() << "-->" << vfh_mpc.get_goal_posf() << "\n";
	//ゴールセルに到達したら終了
	cv::Point xri = vfh_mpc.get_posi();
	cv::Point xgi = vfh_mpc.get_goal_posi();
	if (xri.x == xgi.x && xri.y == xgi.y)
	{
		std::cout << "Goal\n";
		return false;
	}
	//MPC
	ROS_INFO("get_speed");
	v0 = vfh_mpc.get_speed(vfh_mpc.get_posf(), vfh_mpc.get_vel());
	std::cout<<"v0:"<<v0<<"\n";
	vfh_mpc.clear_move_data();
	ROS_INFO("add_mv_pot");
	vfh_mpc.add_mv_grid(vfh_mpc.get_grid_map()); 
	ROS_INFO("check_collisin");
	if (vfh_mpc.check_collision(vfh_mpc.get_posf()))//collision
	{
		ROS_INFO("collision...\n");
		return false;
	}

	//ロボットの命令速度算出
	float w, v,angle;
	float cost=0;
	v=v0;
	ROS_INFO("set_command_vel...\n");
	vfh_mpc.set_polar_histogram(vfh_mpc.get_grid_map(),vfh_mpc.get_posf(),vfh_mpc.get_ori());
	angle = vfh_mpc.select_angle(vfh_mpc.get_posf(),cost,vfh_mpc.get_ori());
	vfh_mpc.set_command_vel(vfh_mpc.get_ori(),angle, v, w);
	//cgen.update_RobotVel(v,w);
	std::cout << "v,w:" << v << "," << w << "\n";
	cgen.publish_velocity(v, w);
	cgen.publish_wheel_velocity(v, w);
	vfh_mpc.set_pub_mpc_debug_images(xri);
	return true;
}
void PROCESS_SIMULATION(command_generator& cgen, VFH_MPC& vfh_mpc) {

	//障害物セット
	//静止障害物
	std::cout<<"set static obstacles\n";
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

	//移動障害物
	float wo=0.30;
	float ho=0.30;
	float reso=vfh_mpc.get_reso();

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
	vfh_mpc.set_mv_obstacle_data(mvObst1,vx,vy);
	vfh_mpc.end_set_mv_obstacle_data();
	//ロボットパラメータセット
	//center point
	cv::Point2f cpt=cv::Point2f(0.0,0.0);
	//goal point
	cv::Point2f goal_pt=cv::Point2f(7.5-5,7.5-5);
	//set_param
	std::cout<<"set_param...\n";
	vfh_mpc.set_center_point(cpt.x,cpt.y);
	vfh_mpc.set_goal(goal_pt);

	if(!vfh_mpc.set_robot_param(-2.5,-2.5,0.2,0.2,M_PI/2))//)
	{
		std::cout<<"Error: robot param\n";
		return ; 
	}
	//--set_command_limit(float dif_vel)
	vfh_mpc.set_command_limit(0.1);
	// apf_mpc.set_mov_time(0.1);
	vfh_mpc.set_mov_time(1);
	std::cout<<"vfh_mpc.draw_mpc_path_mat();...\n";
	vfh_mpc.draw_mpc_path_mat();
}