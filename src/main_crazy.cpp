#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h> // CHK
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h> // CHK

#include <Eigen/Dense>
#include <string>
#include "so3_utils.h"

#define PUBLISH_RAW_TWIST true
#define ENABLE_VERBOSE true

#define n 6
#define m 3

using namespace Eigen;
typedef Matrix<double, n, n> FMatrix;
typedef Matrix<double, n, m> GMatrix;
typedef Matrix<double, n, n> QMatrix;
typedef Matrix<double, n, n> PMatrix;
typedef Matrix<double, m, m> RMatrix;
typedef Matrix<double, m, n> HMatrix;
typedef Matrix<double, n, 1> NVector;
typedef Matrix<double, m, 1> MVector;
typedef Matrix<double, 3, 1> VVector; // jdh
typedef Matrix<double, 3, 3> Rotmat; // jdh

FMatrix F = FMatrix::Zero();
GMatrix G = GMatrix::Zero();
QMatrix Q = QMatrix::Zero();
RMatrix R = RMatrix::Zero();
HMatrix H = HMatrix::Zero();

NVector x_old = NVector::Zero();
NVector x_predict = NVector::Zero();
NVector x_estimate = NVector::Zero();
VVector world_velocity = VVector::Zero(); // jdh
VVector body_velocity = VVector::Zero(); // jdh
Rotmat rotation_matrix = Rotmat::Zero(); // jdh
PMatrix P_old = PMatrix::Zero();
PMatrix P_predict = PMatrix::Zero();
PMatrix P_estimate = PMatrix::Zero();

MVector sigma_Q = MVector::Zero();
MVector sigma_R = MVector::Zero();

geometry_msgs::TransformStamped pose_old, pose_new;
geometry_msgs::TwistStamped twist, twist_raw;

ros::Subscriber pose_sub;
ros::Publisher twist_pub, twist_pub_raw;

ros::WallTime t_old, t_new;
bool initialized = false;

void pose_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);

void predict(const double &dt);
FMatrix computeF(const double &dt);
GMatrix computeG(const double &dt);
QMatrix computeQ(const GMatrix &G, const MVector &sigma_Q);
void update(const double &dt, const geometry_msgs::TransformStamped::ConstPtr &msg);
RMatrix computeR();

int main(int argc, char **argv)
{
	sigma_Q(0,0) = 20.0;
	sigma_Q(1,0) = 20.0;
	sigma_Q(2,0) = 20.0;
	sigma_R(0,0) = 0.001;
	sigma_R(1,0) = 0.001;
	sigma_R(2,0) = 0.001;

	H(0,0) = 1.0;
	H(1,1) = 1.0;
	H(2,2) = 1.0;
//////////////////////////////////////////////////////////////////
//
	std::string pose_topic, output_topic, outputRaw_topic;

	ros::init(argc, argv, "vicon_twist");
	ros::NodeHandle nh(ros::this_node::getName());
	ros::Duration(3.0).sleep();

	nh.param<std::string>("pose_topic", pose_topic, pose_topic);
	nh.param<std::string>("output_topic", output_topic, output_topic);
	nh.param<std::string>("outputRaw_topic", outputRaw_topic, outputRaw_topic);
	ROS_INFO_STREAM("Subscribing pose topic : " << pose_topic);
	ROS_INFO_STREAM("Output twist topic : " << output_topic);
	ROS_INFO_STREAM("Output raw twist topic : " << outputRaw_topic);

	pose_sub = nh.subscribe<geometry_msgs::TransformStamped>
								(pose_topic, 10, pose_cb);
	twist_pub = nh.advertise<geometry_msgs::TwistStamped>
								(output_topic, 10);
	twist_pub_raw = nh.advertise<geometry_msgs::TwistStamped>
								(outputRaw_topic, 10);

	ros::Rate rate(1000);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
}

void pose_cb(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	if(initialized)
	{
		t_new = ros::WallTime::now();
		double dt = (t_new - t_old).toSec();
		predict(dt);
		update(dt, msg);

		//// <jdh
		world_velocity(0,0) = x_estimate(3,0);
		world_velocity(1,0) = x_estimate(4,0);		
		world_velocity(2,0) = x_estimate(5,0);
		rotation_matrix = so3::q2R(msg->transform.rotation.w,msg->transform.rotation.x,msg->transform.rotation.y,msg->transform.rotation.z);

		body_velocity = rotation_matrix.transpose() * world_velocity;
		//// jdh>

		ros::WallTime t_now_wall = ros::WallTime::now();
		ros::Time t_now;
		t_now.sec = t_now_wall.sec;
		t_now.nsec = t_now_wall.nsec;
		twist.header.stamp = t_now;
		twist.twist.linear.x = body_velocity(0,0);
		twist.twist.linear.y = body_velocity(1,0);
		twist.twist.linear.z = body_velocity(2,0);
		twist_pub.publish(twist);

			pose_new = *msg;
			twist_raw.header.stamp = t_now;
			twist_raw.twist.linear.x = (pose_new.transform.translation.x-pose_old.transform.translation.x)/dt;
			twist_raw.twist.linear.y = (pose_new.transform.translation.y-pose_old.transform.translation.y)/dt;
			twist_raw.twist.linear.z = (pose_new.transform.translation.z-pose_old.transform.translation.z)/dt;
			twist_pub_raw.publish(twist_raw);


		x_old = x_estimate;
		P_old = P_estimate;
		t_old = t_new;
		pose_old = pose_new;
	}
	else
	{
		t_old = ros::WallTime::now();
		x_old(0,0) = msg->transform.translation.x;
		x_old(1,0) = msg->transform.translation.y;
		x_old(2,0) = msg->transform.translation.z;

		P_old(0,0) = 0.1;
		P_old(1,1) = 0.1;
		P_old(2,2) = 0.1;
		P_old(3,3) = 1.1;
		P_old(4,4) = 1.1;
		P_old(5,5) = 1.1;

		if(PUBLISH_RAW_TWIST) pose_old = *msg;
		initialized = true;
	}
}

void predict(const double &dt)
{
	F = computeF(dt);
	G = computeG(dt);
	Q = computeQ(G, sigma_Q);

	x_predict = F*x_old;
	P_predict = F*P_old*F.transpose() + Q;
}

void update(const double &dt, const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	MVector measure = MVector::Zero();
	measure(0,0) = msg->transform.translation.x;
	measure(1,0) = msg->transform.translation.y;
	measure(2,0) = msg->transform.translation.z;

	MVector residual = measure - H*x_predict;
	RMatrix R = computeR();
	RMatrix innovation = R + H*P_predict*H.transpose();
	GMatrix K = P_predict*H.transpose()*innovation.inverse();

	x_estimate = x_predict + K*residual;
	P_estimate = P_predict - K*innovation*K.transpose();
}

FMatrix computeF(const double &dt)
{
	FMatrix temp = FMatrix::Zero();
	for(int i=0; i<n; i++)
	{
		temp(i,i) = 1.0;
	}
	temp(0,3) = dt;
	temp(1,4) = dt;
	temp(2,5) = dt;

	return temp;
}

GMatrix computeG(const double &dt)
{
	GMatrix temp = GMatrix::Zero();
	for(int i=0; i<3; i++)
	{
		temp(i,i) = 0.5*dt*dt;
		temp(i+3,i) = dt;
	}

	return temp;
}

QMatrix computeQ(const GMatrix &G, const MVector &sigma_Q)
{
	RMatrix temp = RMatrix::Zero();
	for(int i=0; i<m; i++)
	{
		temp(i,i) = sigma_Q(i,0)*sigma_Q(i,0);
	}

	return G*temp*G.transpose();
}

RMatrix computeR()
{
	RMatrix temp = RMatrix::Zero();
	for(int i=0; i<m; i++)
	{
		temp(i,i) = sigma_R(i,0);
	}

	return temp;
}
