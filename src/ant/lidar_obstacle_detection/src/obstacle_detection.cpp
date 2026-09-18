#include "lidar_obstacle_detection/obstacle_detection.h"

using namespace ObstacleDetection_ns;

ObstacleDetection::ObstacleDetection(ros::NodeHandle node_, ros::NodeHandle private_nh)
{

    node = node_;
    cloud_sub = node.subscribe("/lslidar_point_cloud", 10, &ObstacleDetection::cloud_callback,this); //原始点云
    raw_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("raw_cloud_output", 1);
    cloud_pub = node.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);

    box_car_pub = node.advertise<jsk_recognition_msgs::BoundingBox>("object_box_car", 1);
    box_car2_pub = node.advertise<jsk_recognition_msgs::BoundingBox>("object_box_car2", 1);
    box_stop_pub = node.advertise<jsk_recognition_msgs::BoundingBox>("object_box_stop", 1);
    box_range_pub = node.advertise<jsk_recognition_msgs::BoundingBox>("object_box_range", 1);

    box_pub = node.advertise<jsk_recognition_msgs::BoundingBoxArray>("object_box", 10);
    box_pub_ab = node.advertise<jsk_recognition_msgs::BoundingBoxArray>("object_box_ab", 10); // 障碍物
    speed_ratio_pub = node.advertise<std_msgs::Float64>("safe_range_ctrl", 10);               // 障碍物
    current_velocity_sub_ = node.subscribe("/current_velocity", 10, &ObstacleDetection::callbackCurrentVelocity, this);

    private_nh.param<string>("frame_id", frame_id, "velodyne");
    private_nh.param<double>("car_length", car_length, 1.9);
    private_nh.param<double>("car_width", car_width, 1.0);
    private_nh.param<double>("car_hight", car_hight, 1.5);
    private_nh.param<double>("lidar_front_dis", lidar_front_dis, 1.5);
    private_nh.param<double>("front_stop_distance", front_stop_distance, 1.0);
    private_nh.param<double>("edge_stop_distance", edge_stop_distance, 0.25);
    private_nh.param<double>("back_stop_distance", back_stop_distance, 0.1);
    
    //////////////////////////////
    #if 0
    path_dir = "/home/huazai/Git/Mayixia/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_2";
    //Stream PCD
    pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    stream = pointProcessorI->streamPcd(path_dir);
    streamIterator = stream.begin();
    #endif
    
    private_nh.param<double>("x_stop_range_front", x_stop_range_front, 1.5);
    private_nh.param<double>("x_stop_range_rear", x_stop_range_rear, 1.5);
    private_nh.param<double>("y_stop_range", y_stop_range, 0.8);
    private_nh.param<double>("z_stop_range", z_stop_range, 1.81);

    private_nh.param<double>("x_range", x_range, 30);
    private_nh.param<double>("y_range", y_range, 10);
    private_nh.param<double>("z_range", z_range, 1.81);

    private_nh.param<double>("filterRes", filterRes, 0.05);
    private_nh.param<double>("cluster_distance", cluster_distance, 0.3);
    private_nh.param<int>("min_points_num", min_points_num, 3);
    private_nh.param<int>("max_points_num", max_points_num, 500);

    //server = new dynamic_reconfigure::Server<lidar_obstacle_detection::DetectionConfig>(private_nh);
    dynamic_reconfigure::Server<lidar_obstacle_detection::DetectionConfig>::CallbackType f;
    f = boost::bind(&ObstacleDetection::reconfigureCB,this, _1, _2);
    server.setCallback(f);
}

ObstacleDetection::~ObstacleDetection()
{
}

void ObstacleDetection::reconfigureCB(lidar_obstacle_detection::DetectionConfig &config,uint32_t level)
{
    x_stop_range_front =  config.x_stop_range_front;
    x_stop_range_rear =  config.x_stop_range_rear;
    y_stop_range = config.y_stop_range;
    z_stop_range = config.z_stop_range;

    x_range = config.x_range;
    y_range = config.y_range;
    z_range = config.z_range;

    filterRes = config.filterRes;
    cluster_distance = config.cluster_distance;
    min_points_num = config.min_points_num;
    max_points_num = config.max_points_num;

    if (config.save_config == true)
    {
    system("rosparam set  /lidar_obstacle_detect/save_config false");
    system("rosparam dump $(rospack find lidar_obstacle_detection)/params/lidar_euclidean.yaml /lidar_obstacle_detect/");
    // system("rosnode kill /lidar_obstacle_detect");
    }
    if(config.recover_default == true){
    system("rosparam set  /lane_follow/recover_default false");
    system("rosparam load $(rospack find lidar_obstacle_detection)/params/lidar_euclidean_default.yaml /lidar_obstacle_detect/");
    system("rosparam dump $(rospack find lidar_obstacle_detection)/params/lidar_euclidean.yaml /lidar_obstacle_detect/");
    // system("rosnode kill /lidar_obstacle_detect");
    }
}

void ObstacleDetection::callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg)
{
  cur_velocity = msg->twist.linear.x;
  if(cur_velocity < 0)
  {
      cur_velocity =0;
  }else if(cur_velocity > 5){
      cur_velocity =5;
  }
}

bool ObstacleDetection::speed_control(Box box_point,double speed_ratio[])
{

    speed_ratio[0] = speed_ratio[1]= speed_ratio[2] = 1.0;
    bool ret = false;



    // 前面车盖的角度
    double angle = atan2(car_width/2, car_length/2); //+  M_PI / 180 * 10;       //  M_PI / 180 * 10 表示加大一点前视角度
    double x_, y_, z_;

    x_ = abs(box_point.x_max) - abs(box_point.x_min) > 0 ? box_point.x_min : box_point.x_max;
    y_ = abs(box_point.y_max) - abs(box_point.y_min) > 0 ? box_point.y_min : box_point.y_max;
    z_ = abs(box_point.z_max) - abs(box_point.z_min) > 0 ? box_point.z_min : box_point.z_max;
    // box.dimensions.x = box_point.x_max - box_point.x_min;
    // box.dimensions.y = box_point.y_max - box_point.y_min;
    // box.dimensions.z = box_point.z_max - box_point.z_min;

    y_ = abs(y_);
    // 四周停止范围
    if (x_ > -(car_length / 2 + back_stop_distance) &&
        x_ < (car_length / 2 + front_stop_distance + cur_velocity*1.5) && y_ < (car_width / 2 + edge_stop_distance * cur_velocity*0.3)){
        speed_ratio[0] = 0;
        ret = true;
    }
    else{
        // // 前方停止范围乘3倍进行减速
        // if (x_ > (car_length / 2 + back_stop_distance) &&
        //         x_ < (car_length / 2 + front_stop_distance * 3) && y_ < (car_width / 2 + edge_stop_distance))
        // {
        //     //  speed_ratio[1] = x_ - (car_length / 2 + front_stop_distance) / (front_stop_distance * 2); // 7 - 3 / 4, 减速比例 0~1
        //     speed_ratio[1] =  (x_ - (car_length / 2))/  (front_stop_distance*3);  // 7 - 1 / 6                  , 减速比例 0.33~1
        // } //侧方减速
        // if (x_ > -(car_length / 2 + back_stop_distance) &&
        //         x_ < (car_length / 2 + front_stop_distance) && y_ < (car_width / 2 + edge_stop_distance * 3))
        // {
        //     // speed_ratio[2] =  y_ - (car_width / 2 + edge_stop_distance) /  (edge_stop_distance * 2);
        //     speed_ratio[2] = (y_ - (car_width / 2)) / (edge_stop_distance * 3);
        // }
    }
#if 0
    if (-angle < atan2(y_,x_) && atan2(y_,x_) < angle)             // 前视角度   不能使用 1 < x < 2
    {
        if (((car_length / 2) + front_stop_distance) >= sqrt(pow(x_, 2) + pow(y_, 2)))
        {
            // 停止
            printf("--- 前方有障碍物\n");
            speed_ratio[0] = 0;
            ret = true;
        }
        else if (((car_length / 2) + front_stop_distance * 3) > sqrt(pow(x_, 2) + pow(y_, 2)))
        {
            // 减速
            //speed_ratio[0] = (sqrt(pow(x_, 2) + pow(y_, 2)) - (car_length / 2 + front_stop_distance)) / (front_stop_distance * 4); //控制速度的比例
            speed_ratio[0] = (sqrt(pow(x_, 2) + pow(y_, 2)) - (car_length / 2 )) / (front_stop_distance * 3);  // 100~33-0
        }else{
            //速度恢复
            speed_ratio[0] = 1;
        }
    }else if((-angle + M_PI) < (atan2(y_,x_) < 0?atan2(y_,x_)+M_PI*2:atan2(y_,x_)) && (atan2(y_,x_) < 0?atan2(y_,x_)+M_PI*2:atan2(y_,x_)) < (angle + M_PI))    // 后视角度 ，tan在第一、三象限为正，在第二、四为负
    {

        if(-(car_length / 2 + back_stop_distance) <= x_ && x_ <= (car_length / 2 + back_stop_distance))
        {
            printf("--- 后方有障碍物\n");
            speed_ratio[2] = 0;
            ret = true;
        }else if(-(car_length / 2 + back_stop_distance*3) < x_ && x_ < (car_length / 2 + back_stop_distance*3))
        {
            //speed_ratio[2] = (abs(x_) - (car_length / 2 + back_stop_distance)) / (back_stop_distance * 2); //控制速度的比例
            speed_ratio[2] = (abs(x_) - (car_length / 2 )) / (back_stop_distance * 3); 
        }else{
            speed_ratio[2] = 1;
        }
    }
    else // 侧边角度
    {

        if (-(car_width / 2 + edge_stop_distance) <= y_ && y_ <= (car_width / 2 + edge_stop_distance))
        {
            printf("--- 侧方有障碍物\n");
            speed_ratio[1] = 0;
            ret = true;
        }
        else if (-(car_width / 2 + edge_stop_distance * 3) < y_ && y_ < (car_width / 2 + edge_stop_distance * 3))
        {
            //speed_ratio[1] = (abs(y_) - (car_width / 2 + edge_stop_distance)) / (edge_stop_distance * 2); //控制速度的比例
            speed_ratio[1] = (abs(y_) - (car_width / 2))/ (edge_stop_distance * 3);
        }
        else
        {
            speed_ratio[1] = 1;
        }
    }
#endif
    return ret;
}

bool ObstacleDetection::safe_detection(Box box_point,double speed_ratio[])
{

    speed_ratio[0] = speed_ratio[1]= speed_ratio[2] = 1.0;
    bool ret = false;



    // 前面车盖的角度
    double angle = atan2(car_width/2, car_length/2); //+  M_PI / 180 * 10;       //  M_PI / 180 * 10 表示加大一点前视角度
    double x_, y_, z_;

    x_ = abs(box_point.x_max) - abs(box_point.x_min) > 0 ? box_point.x_min : box_point.x_max;
    y_ = abs(box_point.y_max) - abs(box_point.y_min) > 0 ? box_point.y_min : box_point.y_max;
    z_ = abs(box_point.z_max) - abs(box_point.z_min) > 0 ? box_point.z_min : box_point.z_max;
    // box.dimensions.x = box_point.x_max - box_point.x_min;
    // box.dimensions.y = box_point.y_max - box_point.y_min;
    // box.dimensions.z = box_point.z_max - box_point.z_min;

    y_ = abs(y_);
    // 四周停止范围
    if (x_ > -(x_stop_range_rear) &&
        x_ < (x_stop_range_front) && y_ < (y_stop_range) &&
        z_ < (z_stop_range -car_hight)){
        speed_ratio[0] = 0;
        ret = true;
    }
    else{
        // // 前方停止范围乘3倍进行减速
        // if (x_ > (car_length / 2 + x_stop_range_rear) &&
        //         x_ < (car_length / 2 + x_stop_range_front * 3) && y_ < (car_width / 2 + y_stop_range))
        // {
        //     //  speed_ratio[1] = x_ - (car_length / 2 + x_stop_range_front) / (x_stop_range_front * 2); // 7 - 3 / 4, 减速比例 0~1
        //     speed_ratio[1] =  (x_ - (car_length / 2))/  (x_stop_range_front*3);  // 7 - 1 / 6                  , 减速比例 0.33~1
        // } //侧方减速
        // if (x_ > -(car_length / 2 + x_stop_range_rear) &&
        //         x_ < (car_length / 2 + x_stop_range_front) && y_ < (car_width / 2 + y_stop_range * 3))
        // {
        //     // speed_ratio[2] =  y_ - (car_width / 2 + y_stop_range) /  (y_stop_range * 2);
        //     speed_ratio[2] = (y_ - (car_width / 2)) / (y_stop_range * 3);
        // }
    }
    return ret;
}

void ObstacleDetection::pub_speed_ratio(double min)
{
    //发布控制速度比例值
    std_msgs::Float64 mix_msg;
    min = floor(min * 100.000f + 0.5) / 100.000f;
    mix_msg.data = min;
    speed_ratio_pub.publish(mix_msg);
}

void ObstacleDetection::bounding_box(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters)
{
    auto startTime = chrono::high_resolution_clock::now();

    jsk_recognition_msgs::BoundingBox box;
    jsk_recognition_msgs::BoundingBoxArray boxs,boxs_ab;
    int i = 0;

    double min_speed=1.0;
    double min_speed_t;
    double speed_ratio[3];

    //for (int objectI = 0; objectI < visBBs.size(); objectI++)
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        Box box_point = pointProcessorI->BoundingBox(cluster); // 返回 最大和最小点的位置，即对角线的2点
        box.label = i + 1;
        #if 0
        //calculate bounding box
        double length_ = box_point.x_max - box_point.x_min;
        double width_ =  box_point.y_max - box_point.y_min;
        double height_ = box_point.z_max - box_point.z_min;

        box.pose.position.x = box_point.x_min + length_ / 2;
        box.pose.position.y = box_point.y_min + width_ / 2;
        box.pose.position.z = box_point.z_min + height_ / 2;
        box.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);   
        box.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        box.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);
        #else
        box.pose.position.x = (box_point.x_max + box_point.x_min) / 2;
        box.pose.position.y = (box_point.y_max + box_point.y_min) / 2;
        box.pose.position.z = (box_point.z_max + box_point.z_min) / 2;
        box.dimensions.x = box_point.x_max - box_point.x_min;
        box.dimensions.y = box_point.y_max - box_point.y_min;
        box.dimensions.z = box_point.z_max - box_point.z_min;
        #endif

        // if (speed_control(box_point,speed_ratio) == true)
        if (safe_detection(box_point,speed_ratio) == true)
        {
            box.header.frame_id = frame_id;
            boxs_ab.boxes.push_back(box);
        }

        min_speed_t = speed_ratio[0] < speed_ratio[1] ? speed_ratio[0] : speed_ratio[1];     // 四周方向中选择一个最小值的发布
        min_speed_t = min_speed_t < speed_ratio[2] ? min_speed_t : speed_ratio[2];
        if(min_speed > min_speed_t){
            min_speed = min_speed_t;
        }

        box.header.frame_id = frame_id;
        //box.header.stamp = input_msg.header.stamp;
        boxs.boxes.push_back(box);
    }

    //boxs.header.stamp = input_msg.header.stamp;
    boxs.header.frame_id = frame_id;
    box_pub.publish(boxs);
    boxs_ab.header.frame_id = frame_id;
    box_pub_ab.publish(boxs_ab);

    pub_speed_ratio(min_speed);

    auto endTime = chrono::high_resolution_clock::now();
    chrono::duration<double, milli> fp_ms = endTime - startTime;
    cout << fp_ms.count() << endl;
}

void ObstacleDetection::cloud_callback(const sensor_msgs::PointCloud2ConstPtr input)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr elevatedCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZI>());

    // Convert from ros msg to PCL::PointCloud data type
    pcl::fromROSMsg(*input, cloud);

    #if 0
    // 去除地面
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = pointProcessorI->FilterCloud(cloud.makeShared(), 0.1, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 6, 0.5, 1), Eigen::Vector4f(-(car_length / 2), -(car_width / 2), -car_hight, 1.0), Eigen::Vector4f(car_length / 2, car_width / 2, 0, 1.0));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(cloudFiltered, 50, 0.3);

    // 包围框   ----------------------------------
    //clustering 团族集群,点云分割,显示不同的颜色    ;障碍物检测
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.53, 3, 2000); // 0.53米内表示是同一个团族, 2<x<500的团集认为是一个障碍物
#else
    # if 1                                                                                                                                                                                    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = pointProcessorI->FilterCloud(cloud.makeShared(), filterRes, Eigen::Vector4f(-x_range, -y_range, -car_hight, 1.0), Eigen::Vector4f(x_range, y_range, (z_range -car_hight) , 1.0), Eigen::Vector4f(-(car_length - lidar_front_dis), -(car_width / 2.0), -car_hight, 1.0), Eigen::Vector4f(lidar_front_dis, car_width / 2.0, 0, 1.0));
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(cloudFiltered, 50, 0.3);

    // 包围框   ----------------------------------
    //clustering 团族集群,点云分割,显示不同的颜色    ;障碍物检测     (欧式距离分割)
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(cloudFiltered, cluster_distance, min_points_num, max_points_num); // 0.53米内表示是同一个团族, 2<x<500的团集认为是一个障碍物
    #endif

    #if 0
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = pointProcessorI->FilterCloud(cloud.makeShared(), 0.05, Eigen::Vector4f(-20, -10, -1.3, 1), Eigen::Vector4f(50, 10, 1, 1), Eigen::Vector4f(-(car_length / 2), -(car_width / 2), -car_hight, 1.0), Eigen::Vector4f(car_length / 2, car_width / 2, 0, 1.0));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(cloudFiltered, 100, 0.3);

    // 包围框   ----------------------------------
    //clustering 团族集群,点云分割,显示不同的颜色    ;障碍物检测
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.53, 3, 1000); // 0.53米内表示是同一个团族, 2<x<500的团集认为是一个障碍物
    #endif
#endif
    int clusterId = 0;

    // for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) // cloudClusters 一帧点云中识别到了多少个物体
    // {
    //     std::cout << "cloudClusters size " << cloudClusters.size() << endl;
    //     std::cout << "cluster size ";
    //     pointProcessorI->numPoints(cluster); // cluster->points.size() 检测到该一个物体点的个数

    //     //render box;
    //     Box box = pointProcessorI->BoundingBox(cluster); // 返回 最大和最小点的位置

    //     //renderBox(viewer, box, clusterId);               // 框框包围物体
    //     ++clusterId;
    // }
    bounding_box(cloudClusters);
    // ---------------------------------------------------
    //发布处理后的点云
    pcl::toROSMsg(*cloudFiltered, output);
    output.header.frame_id = frame_id;
    output.header.seq = input->header.seq;
    output.header.stamp =  ros::Time::now();     //input->header.stamp;      //时间需要同步

    cloud_pub.publish(output);

//    // 发布车体体积
//     box_car.pose.position.x = 0;
//     box_car.pose.position.y =  0;
//     box_car.pose.position.z = car_hight / 2;
//     box_car.dimensions.x = car_length;
//     box_car.dimensions.y = car_width;
//     box_car.dimensions.z = car_hight;

//     box_car.header.frame_id = "base_link";
//     box_car_pub.publish(box_car);

//    // 发布车体2体积
//     box_car2.pose.position.x = 0.25;
//     box_car2.pose.position.y =  0;
//     box_car2.pose.position.z = 0.47;
//     box_car2.dimensions.x = 0.30;
//     box_car2.dimensions.y = car_width;
//     box_car2.dimensions.z = 0.92;

//     box_car2.header.frame_id =  "base_link";
//     box_car2_pub.publish(box_car2);

       // 发布停止体积
    box_stop.pose.position.x = (x_stop_range_front-x_stop_range_rear) / 2.0;
    box_stop.pose.position.y =  0;
    box_stop.pose.position.z =  (z_stop_range / 2.0) - car_hight;
    box_stop.dimensions.x = x_stop_range_front+x_stop_range_rear;
    box_stop.dimensions.y = y_stop_range*2;
    box_stop.dimensions.z = z_stop_range;

    box_stop.header.frame_id = frame_id;
    box_stop_pub.publish(box_stop);

       // 发布有效处理范围体积
    box_range.pose.position.x = 0;
    box_range.pose.position.y =  0;
    box_range.pose.position.z = (z_range / 2.0) - car_hight;
    box_range.dimensions.x = x_range*2;
    box_range.dimensions.y = y_range*2;
    box_range.dimensions.z = z_range;

    box_range.header.frame_id = frame_id;
    box_range_pub.publish(box_range);
}

// 循环发布话题
bool ObstacleDetection::pool()
{
    sensor_msgs::PointCloud2 output;

    //1 发布读取pcd文件的点云话题
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string()); // 加载pcd点云文件
    streamIterator++;
    if (streamIterator == stream.end())
        streamIterator = stream.begin();
    pcl::toROSMsg(*inputCloudI, output);
    output.header.frame_id = frame_id;
    raw_cloud_pub.publish(output);

    return true;
}
