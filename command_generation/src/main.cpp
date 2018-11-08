#include<command_generator.h>

int main(int argc,char **argv)
{
	ros::init(argc,argv,"command_genaration_apf");
    command_generator cgen;
	//apf param
	float W=5;
	float H=5;
	float reso=0.05;
	APF_MPC apf_mpc(W,H,reso);//10,10,0.1);
    if(cgen.setting_RobotExpCondition(apf_mpc,reso)){
        ROS_INFO("Setting failed!");
        return -1;
    }
	float v0=vrt=0.2;//temp
    while(ros::ok()){
		//subscribe data
        cgen.subscribe_objects();
        cgen.subscribe_odometry();
		//recognize obstacle state
        cgen.clear_safety_status();
        if(cgen.dicriminate_obstacle()){
            
        }
		//the position of robot convert global to grid(int)
        trans_point_f_to_i(xrf,xri);
		std::cout<<"xrf,xgf:"<<xrf<<"-->"<<xgf<<"\n";
		//ゴールセルに到達したら終\
		if(xri.x==xgi.x && xri.y==xgi.y)
		{
			std::cout<<"Goal\n";
			break;
		}
		//MPC
		v0=get_speed(xrf,vrt);
		clear_move_data();
		kk=0;
		while(ros::ok()&&kk++<100){
			set_pub_mpc_debug_images(xri);
		}
		
		clear_move_data();
		add_mv_pot(xri,obst_num);
		set_grad(xri);
		if(check_collision(xrf))//collision
		{
			ROS_INFO("collision...\n");
			break;
		}
		
		//ロボットの命令速度算出
		float w,v;
		ROS_INFO("set_command_vel...\n");
		set_command_vel(xri,v0,v,w,th_t);
    }

	ROS_INFO("Done...\n");
	return 0;
}

