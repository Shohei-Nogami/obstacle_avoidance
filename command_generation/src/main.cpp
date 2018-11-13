#include<command_generator.h>

int main(int argc,char **argv)
{
	ros::init(argc,argv,"command_genaration_apf");
    command_generator cgen;
	//apf param
	float W=8;
	float H=8;
	float reso=0.15;
	APF_MPC apf_mpc(W,H,reso);//10,10,0.1);
	//first odometry 
	std::cout<<"waiting first odometry\n";
    cgen.subscribe_odometry();
    if(!cgen.setting_RobotExpCondition(apf_mpc,reso)){
        ROS_INFO("Setting failed!");
        return -1;
    }
	float v0=0.2;//temp
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
		//update param and init data
		cgen.update_RobotPos(apf_mpc);
		//apf_mpc.clear_mv_obstacle_data();
		apf_mpc.clear_grid_map();
		//recognize obstacle state(
        if(cgen.dicriminate_obstacle()){
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
        apf_mpc.trans_point_f_to_i(apf_mpc.get_posf(),apf_mpc.get_posi());
		std::cout<<"xrf,xgf:"<<apf_mpc.get_posf()<<"-->"<<apf_mpc.get_goal_posf()<<"\n";
		//ゴールセルに到達したら終
		cv::Point xri=apf_mpc.get_posi();
		cv::Point xgi=apf_mpc.get_goal_posi();
		if(xri.x==xgi.x && xri.y==xgi.y)
		{
			std::cout<<"Goal\n";
			break;
		}
		//MPC		
		ROS_INFO("get_speed");
		v0=apf_mpc.get_speed(apf_mpc.get_posf(),apf_mpc.get_vel());
		ROS_INFO("clear_move_data");
		apf_mpc.clear_move_data();
		ROS_INFO("add_mv_pot");
		apf_mpc.add_mv_pot(apf_mpc.get_posi(),apf_mpc.get_obst_num());//int --> ref int
		ROS_INFO("set_grad");
		apf_mpc.set_grad(apf_mpc.get_posi());
		ROS_INFO("check_collision");
		if(apf_mpc.check_collision(apf_mpc.get_posf()))//collision
		{
			ROS_INFO("collision...\n");
			break;
		}
		
		//ロボットの命令速度算出
		float w,v;
		ROS_INFO("set_command_vel...\n");
		apf_mpc.set_command_vel(apf_mpc.get_posi(),v0,v,w,apf_mpc.get_ori());
		//cgen.update_RobotVel(v,w);
		std::cout<<"v,w:"<<v<<","<<w<<"\n";
		cgen.publish_velocity(v,w);
		cgen.publish_wheel_velocity(v,w);
		apf_mpc.set_pub_mpc_debug_images(xri);
		
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

