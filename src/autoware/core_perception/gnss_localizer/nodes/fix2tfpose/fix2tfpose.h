#ifndef _FIX2TFPOSE_H_
#define _FIX2TFPOSE_H_

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <string>
#include <gnss/geo_pos_conv.hpp>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
class Fix2tfPose
{
    private:
        ros::Publisher pose_publisher, gnss_publisher;
        ros::NodeHandle private_nh;
        ros::NodeHandle nh;

        int get_gnss_init_pose;
        double roll = 0, pitch = 0, imu_yaw = 0, uwb_yaw = 0;
        
    public:
        Fix2tfPose();
        ~Fix2tfPose();
        void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg);
        void imu_callback(const sensor_msgs::Imu::Ptr& input);
        void uwb_callback(const geometry_msgs::PoseStampedConstPtr &msg);
        void run();
};
      

#endif