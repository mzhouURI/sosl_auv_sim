#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Accel.h"
#include <tf/tf.h>
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>

float hz=10;
//define auv model constant
double m =31.0;
double Izz = 1.00/12.00*m*(3*0.1*0.1+1);
double a = 0.00004;
double b = -0.015;
double Xudot=-3;
double Yvdot = -31.4;
double Nrdot = -2.6;
double PI = 3.1415926;
nav_msgs::Odometry odometry;
geometry_msgs::Vector3 rpy;
std_msgs::Float32 c_rpm;
std_msgs::Float32 c_fin;


double deg2rad (double degrees) {
    
    return degrees * PI / 180.0;
}

double rad2deg (double radians) {

    return radians *180.0/ PI;
}

void dead_reckon()
{
	double T, L, D, N, Xh, Yh, Nh, xdot, ydot, psi_dot;
	geometry_msgs::Accel accel;
	tf::TransformBroadcaster odom_broadcaster;
	//Thruster
	T = a * pow(c_rpm.data, 2) + b* c_rpm.data * odometry.twist.twist.linear.x;
	//Rudder lift
	L =0.1 * c_fin.data * pow(odometry.twist.twist.linear.x, 2);
	D = (0.11 + 0.003 * pow(c_fin.data, 2)) * pow(odometry.twist.twist.linear.x, 2);
	N = -L *0.5; //1 meter long AUV
	//ROS_INFO("T=%.3f|L=%.3f|D=%.3f | N=%.3f",T, L, D, N);	
	//hydrodynamic damping force
	Xh =  -10.0 * odometry.twist.twist.linear.x *fabs(odometry.twist.twist.linear.x);
	Yh = -400.0 * odometry.twist.twist.linear.y *fabs(odometry.twist.twist.linear.y);
	Nh = -200.0 * odometry.twist.twist.angular.z *fabs(odometry.twist.twist.angular.z);
	//ROS_INFO("Xh=%.3f|Yh=%.3f|Nh=%.3f",Xh, Yh, Nh);
	//estimate the acceleration
	accel.linear.x = 1.0f/(m-Xudot) * (T - D + Xh
				 		- Yvdot * odometry.twist.twist.linear.y*odometry.twist.twist.angular.z
				 		+ m *  odometry.twist.twist.linear.y*odometry.twist.twist.angular.z);
	accel.linear.y = 1.0f/(m-Yvdot) * (L + Yh
				 	   	+ Xudot * odometry.twist.twist.linear.x * odometry.twist.twist.angular.z
						- m * odometry.twist.twist.linear.x * odometry.twist.twist.angular.z);
	accel.angular.z = 1.0f/(Izz-Nrdot) * (N + Nh
						- Xudot * odometry.twist.twist.linear.x * odometry.twist.twist.linear.y
						+ Yvdot * odometry.twist.twist.linear.x * odometry.twist.twist.linear.y);
	//ROS_INFO("vx=%.3f|vy=%.3f|vz=%.3f",accel.linear.x,accel.linear.y,accel.angular.z);					
	//update the odomoetry twist
	odometry.twist.twist.linear.x += accel.linear.x*1.0/hz;
	odometry.twist.twist.linear.y += accel.linear.y*1.0/hz;
	odometry.twist.twist.angular.z += accel.angular.z*1.0/hz;
	ROS_INFO("vx=%.3f|vy=%.3f|rz=%.3f",odometry.twist.twist.linear.x,
					   odometry.twist.twist.linear.y,
					   odometry.twist.twist.angular.z);
	//update position and orientation
	//get RPY
	tf::Quaternion q(odometry.pose.pose.orientation.x,
			 odometry.pose.pose.orientation.y,
			 odometry.pose.pose.orientation.z,
			 odometry.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(rpy.x, rpy.y, rpy.z); ///RPY in rad
	if(rpy.z<0)
	{
	rpy.z=rpy.z+2*PI;
	}
	if(rpy.z>2*PI)
	{
	rpy.z=rpy.z-2*PI;
	}
	//ROS_INFO("rpy.x=%.3f|rpy.y=%.3f|rpy.z=%.3f",rpy.x,
	//				   	    rpy.y,
	//				  	    rpy.z);
	
	xdot =   cos( rpy.z ) * odometry.twist.twist.linear.x 
	       - sin( rpy.z ) * odometry.twist.twist.linear.y;
	ydot =   sin( rpy.z ) * odometry.twist.twist.linear.x 
	       + cos( rpy.z ) * odometry.twist.twist.linear.y;
	psi_dot = odometry.twist.twist.angular.z;

	//ROS_INFO("xdot=%.3f|ydot=%.3f|zdot=%.3f",xdot,
	//				   ydot,
	//				  psi_dot);
	
	odometry.pose.pose.position.x += xdot * 1.0/hz;
	odometry.pose.pose.position.y += ydot * 1.0/hz;	
	rpy.z = rpy.z + psi_dot *1.0/hz;
	//update the odometry orientation 2D case
	odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(rpy.z);

	//transform
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "map";
	odom_trans.child_frame_id = "auv";
	//assign translation and rotatino values
	odom_trans.transform.translation.x = odometry.pose.pose.position.x;
	odom_trans.transform.translation.y = odometry.pose.pose.position.y;
	odom_trans.transform.translation.z = odometry.pose.pose.position.z;
	odom_trans.transform.rotation = odometry.pose.pose.orientation;
	
	//send the transform
	odom_broadcaster.sendTransform(odom_trans);
	//
	odometry.header.stamp = odom_trans.header.stamp;
}

void rpmCallback(const std_msgs::Float32::ConstPtr& msg)
{
	c_rpm.data = msg->data;
}

void finCallback(const std_msgs::Float32::ConstPtr& msg)
{
	c_fin.data = msg->data;
}

int main (int argc, char **argv)
{

	ros::init(argc, argv, "auv_model");
	
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	
	ros::Subscriber rpm_sub = nh.subscribe("/auv/c_rpm", 10, rpmCallback);
	ros::Subscriber fin_sub = nh.subscribe("/auv/c_fin", 10, finCallback);
	ros::Publisher m_pose_pub = nh.advertise<nav_msgs::Odometry>("/auv/odometry", 10);
	ros::Publisher m_rpy_pub = nh.advertise<geometry_msgs::Vector3>("/auv/rpy", 10);
	
	nh_priv.getParam("Hz",hz);


	ros::Rate loop_rate(hz);
	c_fin.data=0;
	c_rpm.data=0;

	//initial pose
	odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	odometry.header.frame_id = "auv";
	//odometry.child_frame_id = "base_link";
	getchar();

	while (ros::ok())
	{
	dead_reckon();
	m_pose_pub.publish(odometry);	
	m_rpy_pub.publish(rpy);
	ros::spinOnce();
	loop_rate.sleep();
	}

	return 0;
}
