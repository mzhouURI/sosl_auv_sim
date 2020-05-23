#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"



double deg2rad (double degrees) {
    double PI = 3.1415926;
    return degrees * PI / 180.0;
}

double rad2deg (double radians) {
    double PI = 3.1415926;
    return radians *180.0/ PI;
}


class SpeedController
{
	std_msgs::Float32 c_rpm;
	double sum_error=0;
	double o_speed=0;
	double ot=0;
	double c_speed = 1;
	double Kp = 800;
	double Ki = 400;
	double Kd = 200;

	public:
	SpeedController()
	{	
		double hz =1;
		pub_ = n_.advertise<std_msgs::Float32>("/auv/c_rpm",10);
		
		sub_ = n_.subscribe("/auv/odometry",10,&SpeedController::speedctlCallback,this);
		//ros::NodeHandle nh_priv("~");
		ros::param::get("/speed_controller/c_speed",c_speed);
		ros::param::get("/speed_controller/Kp", Kp);
		ros::param::get("/speed_controller/Kp", Ki);
		ros::param::get("/speed_controller/Kp", Kd);
		ot = ros::Time::now().toSec();
		ros::Rate loop_rate(hz);
	}

	void speedctlCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		
		double error, delta_error, dt;
		error = c_speed - msg->twist.twist.linear.x; //x-axis speed in the vehicle frame
		dt = ros::Time::now().toSec() - ot;
		//dt =0.1;
		delta_error = (msg->twist.twist.linear.x - o_speed)/dt;
		o_speed = msg->twist.twist.linear.x;
		ot =  ros::Time::now().toSec();
		sum_error += error*dt;
		ROS_INFO("o_speed=%.3f|dt=%.3f|de=%.3f|e=%.3f",o_speed,dt,delta_error,error);
		c_rpm.data = Kp*error + Ki*sum_error + Kd*delta_error;

		if(c_rpm.data>2000)
		{
		c_rpm.data=2000;
		}
		if(c_rpm.data<0)
		{
		c_rpm.data=0;
		}
		pub_.publish(c_rpm);
		
	}
	private:
		ros::NodeHandle n_;
		ros::Publisher pub_;
		ros::Subscriber sub_;

};

int main (int argc, char **argv)
{
	
	
	ros::init(argc, argv, "speed_controller");
	
	SpeedController speedctl;

	ros::spin();

	return 0;
}
