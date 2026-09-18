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


ros::Publisher output_;
ros::Publisher output_imu;
ros::Publisher output_imu2;
ros::Publisher output_gps;
// ros::Publisher output_imu3;

std::vector<std::string> split(const std::string &string)
{
    std::vector<std::string> str_vec_ptr;
    std::string token;
    std::stringstream ss(string);

    while (getline(ss, token, ','))
        str_vec_ptr.push_back(token);

    return str_vec_ptr;
}

void convert(std::vector<std::string> nmea, ros::Time current_stamp)
{
    sensor_msgs::NavSatFix gpsFix;

    static int count = 0;
    gpsFix.header.frame_id = "navsat_link";
    gpsFix.header.seq = ++count;
    gpsFix.header.stamp = ros::Time::now();//current_stamp

    try
    {
        if (nmea.at(0).compare(0, 2, "QQ") == 0)
        {
            orientation_time_ = stod(nmea.at(3));
            roll_ = stod(nmea.at(4)) * M_PI / 180.;
            pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
            yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
            orientation_stamp_ = current_stamp;
            orientation_ready_ = true;
            ROS_INFO("QQ is subscribed.");
        }
        else if (nmea.at(0) == "$PASHR")
        {
            orientation_time_ = stod(nmea.at(1));
            roll_ = stod(nmea.at(4)) * M_PI / 180.;
            pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
            yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
            orientation_ready_ = true;
            ROS_INFO("PASHR is subscribed.");
        }
        else if (nmea.at(0).compare(3, 3, "GGA") == 0)
        {
            position_time_ = stod(nmea.at(1));
            double lat = stod(nmea.at(2));
            double lon = stod(nmea.at(4));
            double h = stod(nmea.at(9));

            if (nmea.at(3) == "S")
                lat = -lat;

            if (nmea.at(5) == "W")
                lon = -lon;

            int gps_qual = stod(nmea.at(6));
            if (gps_qual == 0)
                gpsFix.status.status = gpsFix.status.STATUS_NO_FIX;
            else if (gps_qual == 1)
                gpsFix.status.status = gpsFix.status.STATUS_FIX;
            else if (gps_qual == 2)
                gpsFix.status.status = gpsFix.status.STATUS_SBAS_FIX;
            else if (gps_qual == 4 || gps_qual == 5)
                gpsFix.status.status = gpsFix.status.STATUS_GBAS_FIX;
            else if (gps_qual == 9)
                gpsFix.status.status = gpsFix.status.STATUS_SBAS_FIX;
            else
                gpsFix.status.status = gpsFix.status.STATUS_NO_FIX;

            gpsFix.status.service = gpsFix.status.SERVICE_GPS;
            gpsFix.latitude = lat/100;
            gpsFix.longitude = lon/100;
            gpsFix.altitude = h;

            double hdop = stod(nmea.at(8));
            // ROS_WARN("gggggggggggggggg %f",hdop);
            // gpsFix.position_covariance[0] = 0.00762129;
            // gpsFix.position_covariance[4] = 0.00762129;
            // gpsFix.position_covariance[8] = 0.00762129;

            gpsFix.position_covariance_type = gpsFix.COVARIANCE_TYPE_APPROXIMATED;



            // sensor_msgs::NavSatFix gps_data;
            // gps_data.header.stamp = ros::Time::now();
            // gps_data.header.frame_id = "navsat_link";
            // gps_data.latitude = latitudef;
            // gps_data.longitude = longitudef;
            // gps_data.altitude = altitudef;
            // gps_data.status.status = ins_statusf;
            // gps_data.position_covariance_type = position_type;
            gpsFix.position_covariance[0]=pow(hdop,2);
            gpsFix.position_covariance[4]=pow(hdop,2);
            gpsFix.position_covariance[8]=pow(hdop,2);
            output_gps.publish(gpsFix);


            ROS_INFO("GGA is subscribed.  %f     %f    %f",lat, lon, h);
            output_gps.publish(gpsFix);
        }
        else if (nmea.at(0) == "$GPRMC")
        {
            position_time_ = stoi(nmea.at(1));
            double lat = stod(nmea.at(3));
            double lon = stod(nmea.at(5));
            double h = 0.0;

            if (nmea.at(4) == "S")
                lat = -lat;

            if (nmea.at(6) == "W")
                lon = -lon;

            ROS_INFO("GPRMC is subscribed.");
        }
    }
    catch (const std::exception &e)
    {
        ROS_WARN_STREAM("Message is invalid : " << e.what());
    }
}
void callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
    current_time_ = msg->header.stamp;
    convert(split(msg->sentence), msg->header.stamp);
    ROS_ERROR("Pubsh nmea_msgs data......");
}


void points_callback(const sensor_msgs::PointCloud2::ConstPtr input)
{

    // ROS_ERROR("-----------------------------------------------test");
    static int count = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr left_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*input, *left_local_laser);

    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*left_local_laser, outMsg);
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

    IMUoutMsg.angular_velocity.x = -IMUoutMsg.angular_velocity.x;
    // outMsg.angular_velocity.y = outMsg.angular_velocity.y/100;
    IMUoutMsg.angular_velocity.z = -IMUoutMsg.angular_velocity.z;

    // tf::Quaternion quat;
    // tf::quaternionMsgToTF(outMsg.orientation, quat);
    // double roll, pitch, yaw;//定义存储r\p\y的容器
    // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    // // ROS_ERROR("---roll %f     pitch %f    yaw %f",roll*180/3.1415926,pitch*180/3.1415926,yaw*180/3.1415926);

    // double roll2, pitch2, yaw2;//定义存储r\p\y的容器

    // roll2 = yaw;
    // pitch2 = pitch;
    // yaw2 = -roll;
    // // ROS_ERROR("***roll2 %f     pitch2 %f    yaw2 %f",roll2*180/3.1415926,pitch2*180/3.1415926,yaw2*180/3.1415926);


    // tf::Quaternion q;
    // q.setRPY(roll2, pitch2, yaw2);
    // // #create ros msg
    // tf::Quaternion createQuaternionFromRPY(double roll,double pitch,double yaw);
    // // ROS_ERROR("---x %f     y %f    z %f    w %f",q.x(),q.y(),q.z(),q.w());

    // outMsg.orientation.x = q.x();
    // outMsg.orientation.y = q.y();
    // outMsg.orientation.z = q.z();
    // outMsg.orientation.w = q.w();

    IMUoutMsg.linear_acceleration.x=-IMUoutMsg.linear_acceleration.x;
    // // IMUoutMsg.linear_acceleration.y=0;
    IMUoutMsg.linear_acceleration.z=-IMUoutMsg.linear_acceleration.z;
    flag_num++;
    if(flag_num>100)
        flag_num = 0;
}

//转换成base_link的IMU数据用于GPS建图
void imu_callback2(const sensor_msgs::Imu::Ptr& input)
{
    static int count = 0;

    sensor_msgs::Imu outMsg;
    outMsg = *input;
    outMsg.header.frame_id = "base_link";
    outMsg.header.seq = ++count;
    outMsg.header.stamp = ros::Time::now();


    // outMsg.angular_velocity.x = outMsg.angular_velocity.x/100;
    // outMsg.angular_velocity.y = outMsg.angular_velocity.y/100;
    // outMsg.angular_velocity.z = outMsg.angular_velocity.z/100;

    // tf::Quaternion quat;
    // tf::quaternionMsgToTF(outMsg.orientation, quat);
    // double roll, pitch, yaw;//定义存储r\p\y的容器
    // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    // // ROS_ERROR("---roll %f     pitch %f    yaw %f",roll*180/3.1415926,pitch*180/3.1415926,yaw*180/3.1415926);

    // double roll2, pitch2, yaw2;//定义存储r\p\y的容器

    // roll2 = yaw;
    // pitch2 = pitch;
    // yaw2 = -roll;
    // // ROS_ERROR("***roll2 %f     pitch2 %f    yaw2 %f",roll2*180/3.1415926,pitch2*180/3.1415926,yaw2*180/3.1415926);


    // tf::Quaternion q;
    // q.setRPY(roll2, pitch2, yaw2);
    // // #create ros msg
    // tf::Quaternion createQuaternionFromRPY(double roll,double pitch,double yaw);
    // // ROS_ERROR("---x %f     y %f    z %f    w %f",q.x(),q.y(),q.z(),q.w());

    // outMsg.orientation.x = q.x();
    // outMsg.orientation.y = q.y();
    // outMsg.orientation.z = q.z();
    // outMsg.orientation.w = q.w();

    // // IMUoutMsg.linear_acceleration.x=0;
    // // IMUoutMsg.linear_acceleration.y=0;
    // // IMUoutMsg.linear_acceleration.z=0;
    // // IMUoutMsg.linear_acceleration.z = 9.81;
    // // output_imu.publish(IMUoutMsg);

    output_imu.publish(outMsg);
    // ROS_INFO("Pubsh imu_correct data......");
}

//重新发布200HZ的imu话题
void My_Thread()
{
    ros::Rate loop_rate(500);
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
    // outMsg.position_covariance[0]=0;
    // outMsg.position_covariance[4]=0;
    // outMsg.position_covariance[8]=0;
    output_gpsfix.publish(outMsg);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidar_fusion");
    ros::NodeHandle nh;

    ros::Subscriber gpsfix_sub = nh.subscribe("gps/fix", 10, gpsfix_callback);
    output_gpsfix = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix3", 10);

    ros::Subscriber points_sub = nh.subscribe("points_raw", 10, points_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu_raw", 10, imu_callback);
    ros::Subscriber imu_sub2 = nh.subscribe("imu_mapping", 10, imu_callback2);

    ros::Subscriber nmea_sub = nh.subscribe("nmea_sentence", 100, callbackFromNmeaSentence);
    output_gps = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix2", 10);

    output_ = nh.advertise<sensor_msgs::PointCloud2>("/points_mapping", 10);
    // output_imu = nh.advertise<sensor_msgs::Imu>("/imu_raw1", 10);
    output_imu = nh.advertise<sensor_msgs::Imu>("/imu_correct2", 10);
    output_imu2 = nh.advertise<sensor_msgs::Imu>("/imu_mapping", 10);
    std::thread Thread1(My_Thread); //创建线程


    ros::spin();
    return 0;
}