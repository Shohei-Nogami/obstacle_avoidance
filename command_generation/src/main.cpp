#include<command_generator.h>

bool PROCESS(command_generator& cgen, APF_MPC& apf_mpc);
bool PROCESS(command_generator& cgen, VFH_MPC& vfh_mpc);

float v0=0.2;//temp

int main(int argc,char **argv)
{
	// ros::init(argc,argv,"command_genaration_apf");
	ros::init(argc, argv, "command_genaration_vfh");
    command_generator cgen;
	//apf param
	float W=8;
	float H=8;
	float reso=0.15;
	//USED FUNCTION FLAG
	bool USE_APF_MPC = true;
	bool USE_VFH_MPC = true;

	//first odometry 
	std::cout << "waiting first odometry\n";
	cgen.subscribe_odometry();

	//
	APF_MPC apf_mpc(W, H, reso);//10,10,0.1);
	VFH_MPC vfh_mpc(W, H, reso);//10,10,0.1);

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

	float v_temp=0;
	float w_temp=0;
	cgen.update_RobotVel(v_temp,w_temp);
	std::cout<<"process starts\n";
	//turtlebot setting
	geometry_msgs::Twist turtlebot_vel;
	std_msgs::Empty empty_msg;
	ros::NodeHandle nh,nh2;
	ros::Publisher pub,pub2;
	pub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
	pub2 = nh.advertise<std_msgs::Empty>("mobile_base/commands/reset_odometry", 1);

	turtlebot_vel.linear.x=0.2;
	turtlebot_vel.linear.y=0;
	turtlebot_vel.linear.z=0;
	turtlebot_vel.angular.x=0;
	turtlebot_vel.angular.y=0;
	turtlebot_vel.angular.z=0;

	int i=0;
	ros::Rate r=1;

	while(ros::ok()&&i++<2)
	{
		pub2.publish(empty_msg);
		r.sleep();
	}
	
    while(ros::ok()){
		//subscribe data
		cgen.subscribe_objects();
		cgen.subscribe_odometry();

		if (USE_APF_MPC) {
			if(!PROCESS(cgen, apf_mpc)){
				break;
			}
		}
		else if (USE_VFH_MPC) {
			if(!PROCESS(cgen, vfh_mpc)){
				break;
			}

		}
		//turtlebot vel pub
		//pub.publish(turtlebot_vel);
		
	}
    float v=0;
    float w=0;
	cgen.publish_velocity(v,w);
	turtlebot_vel.linear.x=0;
	pub.publish(turtlebot_vel);
	ROS_INFO("Done...\n");
	return 0;
}
bool PROCESS(command_generator& cgen,APF_MPC& apf_mpc) {
	//update param and init data
	cgen.update_RobotPos(apf_mpc);
	//apf_mpc.clear_mv_obstacle_data();
	apf_mpc.clear_grid_map();
	//recognize obstacle state(
	if (cgen.dicriminate_obstacle()) {
		cgen.set_obstacles(apf_mpc);
	}
	//create potential map
	ROS_INFO("create_pot_map");
	apf_mpc.create_pot_map();
	// int obst_num=(int)obst_pti.size()+mv_data_size;
	ROS_INFO("clear_move_data");
	apf_mpc.clear_move_data();
	//the position of robot convert global to grid(int)
	ROS_INFO("trans_point_f_to_i");
	apf_mpc.trans_point_f_to_i(apf_mpc.get_posf(), apf_mpc.get_posi());
	std::cout << "xrf,xgf:" << apf_mpc.get_posf() << "-->" << apf_mpc.get_goal_posf() << "\n";
	//ゴールセルに到達したら終
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
	//apf_mpc.clear_mv_obstacle_data();
	vfh_mpc.clear_grid_map();
	//recognize obstacle state(
	if (cgen.dicriminate_obstacle()) {
		cgen.set_obstacles(vfh_mpc);
	}
	//create potential map
	//ROS_INFO("create_pot_map");
	//vfh_mpc.create_pot_map();
	// int obst_num=(int)obst_pti.size()+mv_data_size;
	ROS_INFO("clear_move_data");
	vfh_mpc.clear_move_data();
	//the position of robot convert global to grid(int)
	ROS_INFO("trans_point_f_to_i");
	vfh_mpc.trans_point_f_to_i(vfh_mpc.get_posf(), vfh_mpc.get_posi());
	std::cout << "xrf,xgf:" << vfh_mpc.get_posf() << "-->" << vfh_mpc.get_goal_posf() << "\n";
	//ゴールセルに到達したら終
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
	ROS_INFO("clear_move_data");
	vfh_mpc.clear_move_data();
	ROS_INFO("add_mv_pot");
	vfh_mpc.add_mv_grid();
	ROS_INFO("check_collision");
	if (vfh_mpc.check_collision(vfh_mpc.get_posf()))//collision
	{
		ROS_INFO("collision...\n");
		return false;
	}

	//ロボットの命令速度算出
	float w, v;
	float cost=0;
	ROS_INFO("set_command_vel...\n");
	vfh_mpc.set_polar_histogram();
	vfh_mpc.set_command_vel(vfh_mpc.select_angle(cost,vfh_mpc.get_ori()), v, w);
	//cgen.update_RobotVel(v,w);
	std::cout << "v,w:" << v << "," << w << "\n";
	cgen.publish_velocity(v, w);
	cgen.publish_wheel_velocity(v, w);
	vfh_mpc.set_pub_mpc_debug_images(xri);
	return true;
}