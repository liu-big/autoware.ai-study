#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

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

#include <sensor_msgs/PointCloud2.h>

ros::Publisher output_;
std::string parent_frame_;

double one_x;
double one_y;
double one_z;
double one_yaw;
double one_pitch;
double one_roll;

double two_x;
double two_y;
double two_z;
double two_yaw;
double two_pitch;
double two_roll;

int parent_filter_enable;
int one_filter_enable;
int two_filter_enable;
double parent_min_angle;
double parent_max_angle;
double one_min_angle;
double one_max_angle;
double two_min_angle;
double two_max_angle;

//ros回调函数，拼接点云

void callback(const sensor_msgs::PointCloud2::ConstPtr &parent_input, const sensor_msgs::PointCloud2::ConstPtr &child_input)
{
    // parent_frame_ = parent_input->header.frame_id;
    static int count = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr parent_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*parent_input, *parent_local_laser);
    if (parent_filter_enable)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointXYZI point_;
        for (std::size_t i = 0; i < parent_local_laser->size(); ++i)
        {
            double angle = atan2(parent_local_laser->points[i].y, parent_local_laser->points[i].x);
            if (angle > parent_min_angle * 3.14159 / 180. && angle < parent_max_angle * 3.14159 / 180.)
            {
                continue;
            }
            filter_laser->points.push_back(parent_local_laser->points[i]);
        }
        parent_local_laser = filter_laser;
    }
    /*
    // 坐标转换
    Eigen::Affine3f g_letf_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    g_letf_calibration_matrix.translation() << 1.105, -0.575, -0.92;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    g_letf_calibration_matrix.rotate(Eigen::AngleAxisf(-0.785, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr parent_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*parent_local_laser, *parent_calibration_cloud, g_letf_calibration_matrix); // Eigen::Affine3f
    */

    // for (std::size_t i = 0; i < parent_local_laser->size(); ++i)
    // {
    //     parent_local_laser->points[i].intensity = 64;
    // }

    // publishCloudI(&g_parent_calib_point_pub, *parent_calibration_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr child_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*child_input, *child_local_laser);
    if (one_filter_enable)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointXYZI point_;
        for (std::size_t i = 0; i < child_local_laser->size(); ++i)
        {
            double angle = atan2(child_local_laser->points[i].y, child_local_laser->points[i].x);
            if (angle > one_min_angle * 3.14159 / 180. && angle < one_max_angle * 3.14159 / 180.)
            {
                continue;
            }
            filter_laser->points.push_back(child_local_laser->points[i]);
        }
        child_local_laser = filter_laser;
    }

    // 坐标转换
    Eigen::Affine3f g_child_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    g_child_calibration_matrix.translation() << one_x, one_y, one_z;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    g_child_calibration_matrix.rotate(Eigen::AngleAxisf(one_yaw, Eigen::Vector3f::UnitZ()));
    g_child_calibration_matrix.rotate(Eigen::AngleAxisf(one_pitch, Eigen::Vector3f::UnitY()));
    g_child_calibration_matrix.rotate(Eigen::AngleAxisf(one_roll, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr child_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*child_local_laser, *child_calibration_cloud, g_child_calibration_matrix);
    // for (std::size_t i = 0; i < child_calibration_cloud->size(); ++i)
    // {
    //     child_calibration_cloud->points[i].intensity = 128;
    // }

    pcl::PointCloud<pcl::PointXYZI>::Ptr calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *calibration_cloud = *parent_local_laser + *child_calibration_cloud;

    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*calibration_cloud, outMsg);
    outMsg.header.frame_id = parent_frame_;
    outMsg.header.seq = ++count;
    outMsg.header.stamp = ros::Time::now();
    output_.publish(outMsg);
}

void callback(const sensor_msgs::PointCloud2::ConstPtr &parent_input, const sensor_msgs::PointCloud2::ConstPtr &one_input, const sensor_msgs::PointCloud2::ConstPtr &two_input)
{
    // parent_frame_ = parent_input->header.frame_id;
    static int count = 0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr parent_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*parent_input, *parent_local_laser);
    if (parent_filter_enable)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointXYZI point_;
        for (std::size_t i = 0; i < parent_local_laser->size(); ++i)
        {
            double angle = atan2(parent_local_laser->points[i].y, parent_local_laser->points[i].x);
            if (angle > parent_min_angle * 3.14159 / 180. && angle < parent_max_angle * 3.14159 / 180.)
            {
                continue;
            }
            filter_laser->points.push_back(parent_local_laser->points[i]);
        }
        parent_local_laser = filter_laser;
    }
    /*
    // 坐标转换
    Eigen::Affine3f g_parent_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    g_parent_calibration_matrix.translation() << 0.6, -0.1, 0.2;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    g_parent_calibration_matrix.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr parent_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*parent_local_laser, *parent_calibration_cloud, g_parent_calibration_matrix);
    */
    // for (std::size_t i = 0; i < parent_calibration_cloud->size(); ++i)
    // {
    //     parent_calibration_cloud->points[i].intensity = 64;
    // }

    pcl::PointCloud<pcl::PointXYZI>::Ptr one_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*one_input, *one_local_laser);
    if (one_filter_enable)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointXYZI point_;
        for (std::size_t i = 0; i < one_local_laser->size(); ++i)
        {
            double angle = atan2(one_local_laser->points[i].y, one_local_laser->points[i].x);
            if (angle > one_min_angle * 3.14159 / 180. && angle < one_max_angle * 3.14159 / 180.)
            {
                continue;
            }
            filter_laser->points.push_back(one_local_laser->points[i]);
        }
        one_local_laser = filter_laser;
    }

    // 坐标转换
    Eigen::Affine3f g_one_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    g_one_calibration_matrix.translation() << one_x, one_y, one_z;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    g_one_calibration_matrix.rotate(Eigen::AngleAxisf(one_yaw, Eigen::Vector3f::UnitZ()));
    g_one_calibration_matrix.rotate(Eigen::AngleAxisf(one_pitch, Eigen::Vector3f::UnitY()));
    g_one_calibration_matrix.rotate(Eigen::AngleAxisf(one_roll, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr one_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*one_local_laser, *one_calibration_cloud, g_one_calibration_matrix);

    // for (std::size_t i = 0; i < one_calibration_cloud->size(); ++i)
    // {
    //     one_calibration_cloud->points[i].intensity = 64;
    // }

    pcl::PointCloud<pcl::PointXYZI>::Ptr two_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*two_input, *two_local_laser);
    if(two_filter_enable)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointXYZI point_; 
        // double angle=137.0*3.14159/180.;
        for (std::size_t i = 0; i < two_local_laser->size(); ++i)
        {
            // point_.x = two_local_laser->points[i].x * cos(angle) + two_local_laser->points[i].y * sin(angle);
            // point_.y = two_local_laser->points[i].y * cos(angle) - two_local_laser->points[i].x * sin(angle);
            // if (point_.x > 0 && point_.y > 0)
            // {
            //     continue;
            // }
            //求出斜率解析过滤
            double angle = atan2(two_local_laser->points[i].y, two_local_laser->points[i].x);
            if (angle > two_min_angle*3.14159/180. && angle < two_max_angle*3.14159/180.){
                continue;
            }
            filter_laser->points.push_back(two_local_laser->points[i]);
        }
        two_local_laser = filter_laser;
    }

    // 坐标转换
    Eigen::Affine3f g_two_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    g_two_calibration_matrix.translation() << two_x, two_y, two_z;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    g_two_calibration_matrix.rotate(Eigen::AngleAxisf(two_yaw, Eigen::Vector3f::UnitZ()));
    g_two_calibration_matrix.rotate(Eigen::AngleAxisf(two_pitch, Eigen::Vector3f::UnitY()));
    g_two_calibration_matrix.rotate(Eigen::AngleAxisf(two_roll, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr two_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*two_local_laser, *two_calibration_cloud, g_two_calibration_matrix);
    // for (std::size_t i = 0; i < two_calibration_cloud->size(); ++i)
    // {
    //     two_calibration_cloud->points[i].intensity = 128;
    // }

    pcl::PointCloud<pcl::PointXYZI>::Ptr calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *calibration_cloud = *one_calibration_cloud + *two_calibration_cloud ;//+ *parent_calibration_cloud ;
    *calibration_cloud = *calibration_cloud + *parent_local_laser;

    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*calibration_cloud, outMsg);
    outMsg.header.frame_id = parent_frame_;
    outMsg.header.seq = ++count;
    outMsg.header.stamp = ros::Time::now();
    output_.publish(outMsg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidar_fusion");
    ros::NodeHandle p_nh("~");
    ros::NodeHandle nh;
    std::string points_parent_src;
    std::string points_child_one;
    std::string points_child_two;
    std::string out_points_cloud;
    int type;

    p_nh.param<std::string>("parent_frame_", parent_frame_, "velodyne");
    p_nh.param<std::string>("points_parent_src", points_parent_src, "/lidar0/points_raw");
    p_nh.param<std::string>("points_child_one", points_child_one, "/lidar1/points_raw");
    p_nh.param<std::string>("points_child_two", points_child_two, "/lidar2/points_raw");

    p_nh.param<std::string>("out_points_cloud", out_points_cloud, "/out_points_cloud");

    p_nh.param<int>("parent_filter_enable", parent_filter_enable, 0);
    p_nh.param<double>("parent_min_angle", parent_min_angle, 0.0);
    p_nh.param<double>("parent_max_angle", parent_max_angle, 0.0);
    p_nh.param<int>("one_filter_enable", one_filter_enable, 0);
    p_nh.param<double>("one_min_angle", one_min_angle, 0.0);
    p_nh.param<double>("one_max_angle", one_max_angle, 0.0);
    p_nh.param<int>("two_filter_enable", two_filter_enable, 0);
    p_nh.param<double>("two_min_angle", two_min_angle, 0.0);
    p_nh.param<double>("two_max_angle", two_max_angle, 0.0);

    p_nh.param<int>("type", type, 2);

    p_nh.param<double>("one_x", one_x, 0);
    p_nh.param<double>("one_y", one_y, 0);
    p_nh.param<double>("one_z", one_z, 0);
    p_nh.param<double>("one_yaw", one_yaw, 0);
    p_nh.param<double>("one_pitch", one_pitch, 0);
    p_nh.param<double>("one_roll", one_roll, 0);

    p_nh.param<double>("two_x", two_x, 0);
    p_nh.param<double>("two_y", two_y, 0);
    p_nh.param<double>("two_z", two_z, 0);
    p_nh.param<double>("two_yaw", two_yaw, 0);
    p_nh.param<double>("two_pitch", two_pitch, 0);
    p_nh.param<double>("two_roll", two_roll, 0);

    // 需要用message_filter容器对两个话题的数据发布进行初始化，这里不能指定回调函数
    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_parent(nh, points_parent_src, 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_one(nh, points_child_one, 10); //,ros::TransportHints().tcpNoDelay()
    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_two(nh, points_child_two, 10);

    output_ = nh.advertise<sensor_msgs::PointCloud2>(out_points_cloud, 10);
#if 0
    // 将两个话题的数据进行同步
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    // 注意修改 100000同步时间,10
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), points_sub_parent, points_sub_one);
    
    // 指定一个回调函数，就可以实现两个话题数据的同步获取
    sync.registerCallback(boost::bind(&callback, _1, _2));
#else
    // else if (type == 3)
    // {
        // 将两个话题的数据进行同步
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
        // 注意修改 100000同步时间,10
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), points_sub_parent, points_sub_one, points_sub_two);

        // 指定一个回调函数，就可以实现两个话题数据的同步获取
        sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    //}
#endif
    ros::spin();
    return 0;
}
