#ifndef _UiBridge_H_
#define _UiBridge_H_

#include <iostream>
#include <cstdlib>
#include <array>
#include <string>
#include <unistd.h>
#include<cstring>

class UiBridge
{
public:
    UiBridge();
    ~UiBridge();

    enum Sensor_type{
        lidar=0,
        camera=1,
    };

    enum Module_type{
        H_Lidar=0,
        H_Camera=1,
        Cam_LaneLine=2,
        Cam_Identity=3,
        Lidar_Detect=4,
    };
    // 数据源
    enum Data_type{
        Real=0,
        Ros_Bag=1,
        Sim=2,
        Record=3,
    };

   enum Calibration_type{
        Camera=0,
        CamAnLidar=1,
    };
    // set();
    // get();
    bool sensor_status(int type);
    bool  module_status(int type);
    void  module_node(int type,char* topic_name);
    //////////////////////////////////////////////////////////
    // 参数动态调整(算法控件)
    void param_set(bool on);
    // 参二:数据源文件(bag包/视频), 参数三:标定文件
    bool  data_source(int type,std::string file="");
    // 相机内参标定
    void camera_calibration(int row, int col, double spacing);
    // 激光雷达与相机标定
    void camera_lidar_calibration();
    // 视觉识别
    void vision_detect(bool on);
    // 车道检测
    void vision_lane_follo(bool on);
    // 激光雷达检测
    void lidar_detect();
    // 激光雷达坐标系
    void coordinate_system(double x, double y,double z,double roll,double pitch, double yaw);
    // 激光雷达与摄像头融合距离
    void vision_lidar_fusion();
    // 选择标定文件
    void select_calibration(int type,std::string cal_file);
    

private:
    
    std::string Module_topic_name[5] = {
        "/point_raw",
        "/img_raw",
        "defg",
        "defg",
        "defg"
    };

    std::string node_name[5] = {
        "/point_raw",
        "/img_raw",
        "defg",
        "defg",
        "defg"
    };

    std::string sensor_cmd[4] ={
            "roslaunch  $(rospack find startingup_ros)/data/Vehicle/SensorShelf/custom_launch/my_sensing.launch",
            "roslaunch  $(rospack find startingup_ros)/data/Vehicle/SensorShelf/custom_launch/play_bag.launch",
            "roslaunch  $(rospack find startingup_ros)/data/Vehicle/SensorShelf/custom_launch/my_sensing_sim.launch",
            "roslaunch  $(rospack find startingup_ros)/data/Vehicle/SensorShelf/custom_launch/play_video.launch",
    };


    std::string module_cmd[1] ={            
        "roslaunch   $(rospack find startingup_ros)/data/Vehicle/SensorShelf/custom_launch/my_detection.launch",
    };
    std::string _file;

    // 杀死所有节点
    bool kill_node(int type);
};

#endif