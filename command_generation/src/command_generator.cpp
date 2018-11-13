#include<command_generator.h>

#define STATIC_OBSTACLE 0
#define DYNAMIC_OBSTACLE 1
#define MOVING_OBSTACLE 2

command_generator::command_generator()
{
	nhs.setCallbackQueue(&queue);
	sub = nhs.subscribe("filted_objects_info", 1, &command_generator::objects_callback, this);

	nhs2.setCallbackQueue(&queue2);
	sub2 = nhs2.subscribe("robot_odm", 1, &command_generator::odometry_callback, this);

	pub = nhp.advertise<command_generation::select_theta>("command_vel", 1);
	vrt=0;
	wrt=0;
}
command_generator::~command_generator()
{
	//vfh.~vfh_class();
}
//setting
bool command_generator::setting_RobotExpCondition(APF_MPC& apf_mpc,float reso)
{
	//setting
	std::cout<<"wait...\n";
    
    // //----実験時にはcpt.x=robot.x,cpt.y=robot.y+5, 逐次更新
    // command_generation::robot_odm robot_x;
    // robot_x.x=5;
    // robot_x.y=0;
    // robot_x.th=M_PI/2;
    //------ゴールポイントも適宜変更
	//center point
	cv::Point2f cpt=cv::Point2f(robot_odm.x,robot_odm.y+5.0);//5.0==map_hf/2
	//goal point
	cv::Point2f goal_pt=cv::Point2f(robot_odm.x,robot_odm.y+10.0);//10.0==map_hf/2
	//-----
	std::cout<<"cpt:"<<cpt<<"\n";
	std::cout<<"goal_pt:"<<goal_pt<<"\n";
	//set_param
	std::cout<<"set_param...\n";
	apf_mpc.set_center_point(cpt.x,cpt.y);
	apf_mpc.set_goal(goal_pt);
	//--set_robot_param(float x,float y, float r,float vt0,float th_t0)
	if(!apf_mpc.set_robot_param(robot_odm.x,robot_odm.y,0.2,0.2,robot_odm.th+M_PI/2))//)
	{
		std::cout<<"Error: robot param\n";
		return false; 
	}
	//--set_command_limit(float dif_vel)
	apf_mpc.set_command_limit(0.1);
	// apf_mpc.set_mov_time(0.1);
	apf_mpc.set_mov_time(0.4);
	return true;	
}
bool command_generator::update_RobotPos(APF_MPC& apf_mpc)
{
    //center point
	cv::Point2f cpt=cv::Point(robot_odm.x,robot_odm.y+5.0);//5.0==map_hf/2
	apf_mpc.set_center_point(cpt.x,cpt.y);
	//--set_robot_param(float x,float y, float r,float vt0,float th_t0)
	// if(!apf_mpc.set_robot_param(robot_odm.x,robot_odm.y,0.2,0.2,robot_odm.th))//)
	if(!apf_mpc.update_robot_param(robot_odm.x,robot_odm.y,robot_odm.th+M_PI/2,apf_mpc.get_vel(),apf_mpc.get_ang_vel()))
	{
		std::cout<<"Error: robot param\n";
		return false; 
	}
	return true;
}
void command_generator::update_RobotVel(float& v,float& w){
	vrt=v;
	wrt=w;
}
float& command_generator::get_vel(void){
	return vrt;
}
float& command_generator::get_angVel(void){
	return wrt;
}
void command_generator::set_pos(cv::Point2f& x){
	x.x=robot_odm.x;
	x.y=robot_odm.y;
}
//subscribe
void command_generator::subscribe_objects(void)
{
	queue.callOne(ros::WallDuration(1));
}
void command_generator::objects_callback(const command_generation::filted_objects_info::ConstPtr& msg)
{
	obj_info.objs = msg->objs;
}
void command_generator::subscribe_odometry(void)
{
	queue2.callOne(ros::WallDuration(100));
}
void command_generator::odometry_callback(const command_generation::robot_odm::ConstPtr& msg)
{
	robot_odm.x=msg->x;
	robot_odm.y=msg->y;
	robot_odm.th=msg->th;
}

//set obstacle status
bool command_generator::dicriminate_obstacle(void)
{
	if(!obj_info.objs.size())
	{
		return false;
	}
	obstacle_status.resize(obj_info.objs.size());
	for (int i = 0; i < obj_info.objs.size(); i++)
	{


		if (obj_info.objs[i].size > 0.7*0.7 || obj_info.objs[i].size < 0.2*0.2
			|| std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0)) < 0.1
			)
		{
			obstacle_status[i] = STATIC_OBSTACLE;
		}
		else if (std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0)) > 1.1
			|| std::sqrt(obj_info.objs[i].vdsp.x + obj_info.objs[i].vdsp.z)
				> std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0)) )
		{
			obstacle_status[i] = DYNAMIC_OBSTACLE;
		}
		else
		{
			obstacle_status[i] = MOVING_OBSTACLE;
		}

	}
	return true;
}
//set obstacle data
void command_generator::set_obstacles(APF_MPC& apf_mpc){
    
    for (int i = 0; i < obj_info.objs.size(); i++)
	{
        if(obstacle_status[i] == MOVING_OBSTACLE){
            //set moving obstacles
            std::vector<cv::Point2f> pts;
            pts.resize(obj_info.objs[i].pt.size());
            for(int k=0;k<obj_info.objs[i].pt.size(); k++){
                pts[k].x=obj_info.objs[i].pt[k].x+robot_odm.x;
                pts[k].y=obj_info.objs[i].pt[k].z+robot_odm.y;
            }
            float vx=obj_info.objs[i].vel.x;
            float vy=obj_info.objs[i].vel.z;
            apf_mpc.set_mv_obstacle_data(pts,vx,vy);
        }
        else{
            //set static obstacles
            for(int k=0;k<obj_info.objs[i].pt.size(); k++){
                cv::Point2f obst_data;
                obst_data.x=obj_info.objs[i].pt[k].x+robot_odm.x;
                obst_data.y=obj_info.objs[i].pt[k].z+robot_odm.y;
	        	apf_mpc.set_static_obstacle_data(obst_data);
            }
        }
    }
}

//publish
void command_generator::publish_velocity(float& v,float w)
{
	command_generation::select_theta pub_data;
	pub_data.select_theta=w;
	pub_data.select_vel=v;
	pub.publish(pub_data);
}
