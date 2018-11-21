#include"ros/ros.h"
#include<fstream>
#include <ros/callback_queue.h>
#include<nav_msgs/Odometry.h>
#include<data_proc/select_theta.h>
#include<data_proc/robot_odm.h>

class data_prcs_exp
{

	ros::NodeHandle nh1,nh2,nh3;
	ros::NodeHandle nh01;
//subscriber
	ros::Subscriber sub1,sub2,sub3;
	ros::Publisher pub1;
	ros::CallbackQueue queue1,queue2,queue3;
    ros::Time t0;
    ros::Duration time_obst;
    
    bool PROCESS_ONCE_OODOM,PROCESS_ONCE_RODOM,PROCESS_ONCE_RVEL;
	nav_msgs::Odometry oodom,oodom0;
	nav_msgs::Odometry dif_odom;
	float dif_th;
	data_proc::robot_odm rodom;
	data_proc::select_theta stheta;
	std::string userpath,filepath;
	
public:	
	data_prcs_exp()
        :PROCESS_ONCE_OODOM(false),PROCESS_ONCE_RODOM(false),PROCESS_ONCE_RVEL(false)
        ,userpath("/home/ros-robot/"),filepath("ex_data/test/"),dif_th(-M_PI)//*5/6)
		{
        //init odometry of an obstacls
		oodom.pose.pose.position.x=0;
		oodom.pose.pose.position.y=0;
		oodom.pose.pose.position.z=0;
		oodom.twist.twist.linear.x=0;
		oodom.twist.twist.linear.y=0;
		oodom.twist.twist.linear.z=0;
		oodom.twist.twist.angular.x=0;
		oodom.twist.twist.angular.y=0;
		oodom.twist.twist.angular.z=0;
        //robot odom
		rodom.x=0;		
		rodom.y=0;		
		rodom.th=0;		
        //dif odom between robot and obstacle
        dif_odom.pose.pose.position.x=0;
		dif_odom.pose.pose.position.y=0;
		dif_odom.pose.pose.position.z=0;
        //vel
		stheta.select_vel=0;
		stheta.select_theta=0;
		nh1.setCallbackQueue(&queue1);
		nh2.setCallbackQueue(&queue2);
		nh3.setCallbackQueue(&queue3);
		sub1=nh1.subscribe("/odom",1,&data_prcs_exp::oodom_callback,this);
		sub2=nh2.subscribe("/robot_odm",1,&data_prcs_exp::robot_odm_callback,this);
		sub3=nh3.subscribe("/select_theta",1,&data_prcs_exp::select_theta_callback,this);
		pub1=nh01.advertise<data_proc::robot_odm>("obstacle_odm",1);
	}
	~data_prcs_exp(){
		
	}
    void set_dif_odom(float x,float y,float z){
        dif_odom.pose.pose.position.x=x;
		dif_odom.pose.pose.position.y=y;
		dif_odom.pose.pose.position.z=z;        
    }
	void oodom_callback(const nav_msgs::Odometry::ConstPtr& msg)
	{
        oodom.header=msg->header;
		oodom.pose=msg->pose;
        oodom.twist=msg->twist;
		oodom.pose.pose.position.x+=dif_odom.pose.pose.position.x;
		oodom.pose.pose.position.y+=dif_odom.pose.pose.position.y;
		oodom.pose.pose.position.z+=dif_odom.pose.pose.position.z;
        
        if(!PROCESS_ONCE_OODOM){
            t0=oodom.header.stamp;
			oodom0=oodom;
            PROCESS_ONCE_OODOM=true;
        }
        else{
            time_obst=oodom.header.stamp-t0;
        }
    }
	void robot_odm_callback(const data_proc::robot_odm::ConstPtr& msg)
	{
		rodom.x=msg->x;		
		rodom.y=msg->y;		
		rodom.th=msg->th;
        if(!PROCESS_ONCE_RODOM){
            PROCESS_ONCE_RODOM=true;
        }
	}
	void select_theta_callback(const data_proc::select_theta::ConstPtr& msg)
	{
		stheta.select_vel=msg->select_vel;
		stheta.select_theta=msg->select_theta;
        if(!PROCESS_ONCE_RVEL){
            PROCESS_ONCE_RVEL=true;
        }
    }
	void call(void){
			queue1.callOne(ros::WallDuration(0.02));
			queue2.callOne(ros::WallDuration(0.02));
			queue3.callOne(ros::WallDuration(0.02));
	}
    void set_label(void){
        //position of an obstacle
		//ros-robot is username
        std::ofstream ofs1(userpath+filepath+"obstacle_pos.csv",std::ios::app);//("./Documents/data_ex.csv",std::ios::app);
		ofs1<<"time"<<","
			<<"x"<<","
			<<"y"<<","
			<<"z"<<","
			<<std::endl;
        //twist of an obstacle
        std::ofstream ofs2(userpath+filepath+"obstacle_twist.csv",std::ios::app);//("./Documents/data_ex.csv",std::ios::app);
		ofs2<<"time"<<","
			<<"vx"<<","
			<<"vy"<<","
			<<"vz"<<","
            <<"ax"<<","
			<<"ay"<<","
			<<"az"<<","<<std::endl;
        //position of the robot
        std::ofstream ofs3(userpath+filepath+"robot_pos.csv",std::ios::app);//("./Documents/data_ex.csv",std::ios::app);
		ofs3<<"time"<<","
			<<"x"<<","
			<<"y"<<","
			<<"th"<<","
			<<std::endl;
        //velocity of the robot
        std::ofstream ofs4(userpath+filepath+"robot_vel.csv",std::ios::app);//("./Documents/data_ex.csv",std::ios::app);
		ofs4<<"time"<<","
			<<"v"<<","
			<<"w"<<","
			<<std::endl;
    }
	bool all_subscribed(void){
		// std::cout<<PROCESS_ONCE_OODOM << "&&"<< PROCESS_ONCE_RODOM<<" && "<<PROCESS_ONCE_RVEL<<"\n";
		if(PROCESS_ONCE_OODOM && PROCESS_ONCE_RODOM ){//&& PROCESS_ONCE_RVEL){
			return true;
		}
		else{
			return false;
		}
	}
	void fprint_data(void){
        //position of an obstacle
        std::ofstream ofs1(userpath+filepath+"obstacle_pos.csv",std::ios::app);//("./Documents/data_ex.csv",std::ios::app);
		
		float dis=std::sqrt((oodom.pose.pose.position.x-oodom0.pose.pose.position.x)*(oodom.pose.pose.position.x-oodom0.pose.pose.position.x)
							+(oodom.pose.pose.position.y-oodom0.pose.pose.position.y)*(oodom.pose.pose.position.y-oodom0.pose.pose.position.y));
//		float dtheta=oodom.pose.pose.orientation.z;
		float xot=dis*cos(dif_th)+dif_odom.pose.pose.position.y;
		float yot=dis*sin(dif_th)+dif_odom.pose.pose.position.x;
		ofs1<<time_obst.toSec()<<","
			// <<oodom.pose.pose.position.x<<","
			// <<oodom.pose.pose.position.y<<","
			// <<oodom.pose.pose.position.z<<","
			<<xot<<","
			<<yot<<","
			<<0<<","
			<<std::endl;
        //twist of an obstacle
        std::ofstream ofs2(userpath+filepath+"obstacle_twist.csv",std::ios::app);//("./Documents/data_ex.csv",std::ios::app);
		ofs2<<time_obst.toSec()<<","
			<<oodom.twist.twist.linear.x<<","
			<<oodom.twist.twist.linear.y<<","
			<<oodom.twist.twist.linear.z<<","
			<<oodom.twist.twist.angular.x<<","
			<<oodom.twist.twist.angular.y<<","
			<<oodom.twist.twist.angular.z<<","
			<<std::endl;
        //position of the robot
        std::ofstream ofs3(userpath+filepath+"robot_pos.csv",std::ios::app);//("./Documents/data_ex.csv",std::ios::app);
		ofs3<<time_obst.toSec()<<","
			<<rodom.x<<","
			<<rodom.y<<","
			<<rodom.th<<","
			<<std::endl;
        //velocity of the robot
        std::ofstream ofs4(userpath+filepath+"robot_vel.csv",std::ios::app);//("./Documents/data_ex.csv",std::ios::app);
		ofs4<<time_obst.toSec()<<","
			<<stheta.select_vel<<","
			<<stheta.select_theta<<","
			<<std::endl;

	}
	void publish_oodom(void){
		data_proc::robot_odm odm;
		float dis=std::sqrt((oodom.pose.pose.position.x-oodom0.pose.pose.position.x)*(oodom.pose.pose.position.x-oodom0.pose.pose.position.x)
							+(oodom.pose.pose.position.y-oodom0.pose.pose.position.y)*(oodom.pose.pose.position.y-oodom0.pose.pose.position.y));
		std::cout<<"dis:"<<dis<<"\n";
		std::cout<<"oodom0:"<<oodom0.pose.pose.position<<"\n";
		std::cout<<"oodom:"<<oodom.pose.pose.position<<"\n";
		float xot=dis*cos(dif_th)+dif_odom.pose.pose.position.x;
		float yot=dis*sin(dif_th)+dif_odom.pose.pose.position.y;
		
		odm.x=xot;
		odm.y=yot;
		odm.th=dif_th;
		pub1.publish(odm);
	}
};

	int main(int argc,char **argv){
		ros::init(argc,argv,"data_proc");
		
		data_prcs_exp prc;
        // prc.set_label();
        prc.set_dif_odom(2.0,3.5,0);		
		// prc.set_dif_odom(2.0,4.0,0);		
		while(ros::ok()){
			prc.call();
			if(prc.all_subscribed()){
				// prc.fprint_data();
				prc.publish_oodom();
			}
		}
		return 0;
}
