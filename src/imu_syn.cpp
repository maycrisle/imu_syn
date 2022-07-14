#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <sensor_msgs/Imu.h>
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> syncPolicy;
ros::Publisher pub_IMU;

void multi_callback(const sensor_msgs::ImuConstPtr &imu_msg_accel, const sensor_msgs::ImuConstPtr &imu_msg_gyro)
{
	std::cout << "同步完成！" << std::endl;
	double t = imu_msg_accel->header.stamp.toSec();
	double dx = imu_msg_accel->linear_acceleration.x;
	double dy = imu_msg_accel->linear_acceleration.y;
	double dz = imu_msg_accel->linear_acceleration.z;
	double rx = imu_msg_gyro->angular_velocity.x;
	double ry = imu_msg_gyro->angular_velocity.y;
	double rz = imu_msg_gyro->angular_velocity.z;
	sensor_msgs::Imu imu_data;

	// 以实时发布的方式发布
	imu_data.header.stamp = imu_msg_accel->header.stamp;
	imu_data.linear_acceleration.x = dx;
	imu_data.linear_acceleration.y = dy;
	imu_data.linear_acceleration.z = dz;
	// 角速度
	imu_data.angular_velocity.x = rx;
	imu_data.angular_velocity.y = ry;
	imu_data.angular_velocity.z = rz;
	// 发布
	pub_IMU.publish(imu_data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_syn");
	ros::NodeHandle n;

	pub_IMU = n.advertise<sensor_msgs::Imu>("imu0", 1000); // 声明发布的话题名称和形式
	message_filters::Subscriber<sensor_msgs::Imu> sub_imu_accel(n, "/camera/accel/sample", 1000, ros::TransportHints().tcpNoDelay()); // 加速度计的话题名称
	message_filters::Subscriber<sensor_msgs::Imu> sub_imu_gyro(n, "/camera/gyro/sample", 1000, ros::TransportHints().tcpNoDelay()); // 陀螺仪的话题名称

	message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_imu_accel, sub_imu_gyro);
	sync.registerCallback(boost::bind(&multi_callback, _1, _2));

	std::cout << "准备接收数据" << std::endl;

	ros::spin();
	return 0;
}
