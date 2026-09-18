#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <memory>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include<thread>
#include <nmea_msgs/Sentence.h>
#include <imu_conversion/imu_conversion.h>
#include <pcl/filters/filter.h>

ros::Publisher output_;
ros::Publisher output_imu;
ros::Publisher output_imu2;
ros::Publisher output_gps;
// ros::Publisher output_imu3;



void points_callback(const sensor_msgs::PointCloud2::ConstPtr input)
{

    // ROS_ERROR("-----------------------------------------------test");
    static int count = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*input, *local_laser);

    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*local_laser, *local_laser, mapping); // 移除无效点，其中 cloud_in 为初始点云，cloud_out 为去除无效点后的点云


    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*local_laser, outMsg);
    outMsg.header.frame_id = "velodyne";
    outMsg.header.seq = ++count;
    outMsg.header.stamp = ros::Time::now();
    output_.publish(outMsg);
    // ROS_INFO("Pubsh points_mapping data......");
}

//接收imu原始话题进行转换
static int flag_num=0;
static sensor_msgs::Imu IMUoutMsg;
void imu_callback(const sensor_msgs::Imu::Ptr& input)
{
    static int count = 0;
    // ROS_ERROR("***************************************imu");
    IMUoutMsg = *input;
    IMUoutMsg.header.frame_id = "imu";
    IMUoutMsg.header.seq = ++count;
    IMUoutMsg.header.stamp = ros::Time::now();

    IMUoutMsg.angular_velocity.x = IMUoutMsg.angular_velocity.x/100;
    IMUoutMsg.angular_velocity.y = IMUoutMsg.angular_velocity.y/100;
    IMUoutMsg.angular_velocity.z = IMUoutMsg.angular_velocity.z/100;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(IMUoutMsg.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    // ROS_ERROR("---roll %f     pitch %f    yaw %f",roll*180/3.1415926,pitch*180/3.1415926,yaw*180/3.1415926);

    double roll2, pitch2, yaw2;//定义存储r\p\y的容器

    roll2 = yaw;
    pitch2 = pitch;
    yaw2 = -roll;
    // ROS_ERROR("***roll2 %f     pitch2 %f    yaw2 %f",roll2*180/3.1415926,pitch2*180/3.1415926,yaw2*180/3.1415926);


    tf::Quaternion q;
    q.setRPY(roll2, pitch2, yaw2);
    // #create ros msg
    tf::Quaternion createQuaternionFromRPY(double roll,double pitch,double yaw);
    // ROS_ERROR("---x %f     y %f    z %f    w %f",q.x(),q.y(),q.z(),q.w());

    IMUoutMsg.orientation.x = q.x();
    IMUoutMsg.orientation.y = q.y();
    IMUoutMsg.orientation.z = q.z();
    IMUoutMsg.orientation.w = q.w();

    // IMUoutMsg.linear_acceleration.x=0;
    // IMUoutMsg.linear_acceleration.y=0;
    // outMsg.linear_acceleration.z=0;
    // IMUoutMsg.linear_acceleration.z = 9.81;
    // output_imu.publish(IMUoutMsg);
    flag_num++;
    if(flag_num>100)
        flag_num = 0;
}


//重新发布200HZ的imu话题
void My_Thread()
{
    ros::Rate loop_rate(200);
    while(ros::ok)
    {
        loop_rate.sleep();
        static int count = 0, last_num, i = 0;
        // ROS_ERROR("***************************************imu");
        if(flag_num != last_num)
        {
            last_num = flag_num;
            i = 0;
        }
        if(last_num == flag_num)
        {
            i++;
        }

        if(i>10)
            ROS_WARN("Waiting imu_raw topic!");
        else
        {
            sensor_msgs::Imu outMsg;
            outMsg = IMUoutMsg;
            // outMsg = *input;
            outMsg.header.frame_id = "imu";
            outMsg.header.seq = ++count;
            outMsg.header.stamp = ros::Time::now();

            output_imu2.publish(outMsg);
            // ROS_INFO("Pubsh imu_mapping data......");
        }
    }
}


ros::Publisher output_gpsfix;
//转换成base_link的IMU数据用于GPS建图
void gpsfix_callback(const sensor_msgs::NavSatFix::Ptr& input)
{
    sensor_msgs::NavSatFix outMsg;
    outMsg = *input;

    static int count = 0;
    outMsg.header.seq = ++count;
    outMsg.header.stamp = ros::Time::now();

    output_gpsfix.publish(outMsg);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidar_fusion");
    ros::NodeHandle nh;

    ros::Subscriber gpsfix_sub = nh.subscribe("fix", 10, gpsfix_callback);
    output_gpsfix = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);

    ros::Subscriber points_sub = nh.subscribe("points_raw", 10, points_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu_raw", 10, imu_callback);


    output_ = nh.advertise<sensor_msgs::PointCloud2>("/points_mapping", 10);

    output_imu2 = nh.advertise<sensor_msgs::Imu>("/imu_mapping", 10);
    std::thread Thread1(My_Thread); //创建线程


    ros::spin();
    return 0;
}