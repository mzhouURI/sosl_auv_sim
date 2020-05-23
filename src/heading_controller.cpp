#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"


double PI = 3.1415926;
double deg2rad (double degrees) {
    
    return degrees * PI / 180.0;
}

double rad2deg (double radians) {

    return radians *180.0/ PI;
}


class HeadingController
{
	std_msgs::Float32 c_fin;
	double sum_error=0;
	double o_speed=0;
	double ot=0;
	double c_heading = 0; //radian
	double Kp = 200;
	double Ki = 20;
	double Kd = 500;
	double m_heading=0;
	public:
	HeadingController()
	{	
		double hz =1;
		pub_ = n_.advertise<std_msgs::Float32>("/auv/c_fin",10);
		
		sub_ = n_.subscribe("/auv/odometry",10,&HeadingController::headingctlCallback,this);
		sub2_= n_.subscribe("/auv/rpy",10, &HeadingController::headingcallback,this);
		sub3_= n_.subscribe("/auv/control/heading",10,&HeadingController::cheadingcallback,this);
		//ros::NodeHandle nh_priv("~");
		//ros::param::get("/heading_controller/c_heading",c_heading);
		ros::param::get("/heading_controller/Kp", Kp);
		ros::param::get("/heading_controller/Ki", Ki);
		ros::param::get("/heading_controller/Kd", Kd);
		//nh_priv.getParam("/speed_controller/c_speed",c_speed);
		ot = ros::Time::now().toSec();
		ros::Rate loop_rate(hz);
	}
	void headingcallback(const geometry_msgs::Vector3::ConstPtr& msg)
	{
		m_heading = msg->z;
		if(m_heading<0)
		{
			m_heading = m_heading+2*PI;
		}
		if(m_heading>2*PI)
		{
			m_heading = m_heading-2*PI;
		}
		
	}
	void cheadingcallback(const std_msgs::Float32::ConstPtr& msg)
	{
		c_heading = msg->data;
	}
	void headingctlCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		double error, delta_error, dt;
		error=0;
		error = c_heading - m_heading; //x-axis speed in the vehicle frame
		//regulate the error		
		if( fabs(error) >PI )
		{
			if(error>0) 
			{
			error = error - 2*PI; 
			}
			else
			{
			error = error + 2*PI;
			}
		}
	
		dt = 0.1;// ros::Time::now().toSec() - ot;
		delta_error = -msg->twist.twist.angular.z;	//error rate is the -angular rate
		ot =  ros::Time::now().toSec();
		sum_error += error*dt;
		ROS_INFO("e=%.3f",error);
		c_fin.data = - (Kp*error + Ki*sum_error + Kd*delta_error);
		
		if(c_fin.data>20)
		{
		c_fin.data=20;
		}
		if(c_fin.data<-20)
		{
		c_fin.data=-20;
		}
		//c_fin.data=0;
		pub_.publish(c_fin);
		
	}
	private:
		ros::NodeHandle n_;
		ros::Publisher pub_;
		ros::Subscriber sub_;
		ros::Subscriber sub2_;
		ros::Subscriber sub3_;

};

int main (int argc, char **argv)
{
	
	
	ros::init(argc, argv, "heading_controller");
	
	HeadingController headingctl;

	ros::spin();

	return 0;
}
