#include "bridge.hpp"



int main(void)
{
    std::string bag_file="/home/ant/work/autoware.ai/build_pcd_data.bag";
    std::string video_file="/tmp/a.mp4";
    std::string cal_file="/home/ant/work/autoware.ai/catkin-ws/src/startingup_ros/data/Vehicle/sim/calibration/autoware_camera_lidar_calibration.yaml";
    std::string name;
    std::string cal_is;
    std::string file_="";
    int num;

    UiBridge *ub = new UiBridge;
    std::cout<<"0 Real , 1 bag , 2 sim , 3 record"<<std::endl;

    std::cout<<"请输入数据源(0/1/2/3): ";
    std::cin>>name;
    switch (atoi(name.c_str()))
    {
        case 0:
        num = ub->Data_type::Real;
        break;

        case 1:
        num = ub->Data_type::Ros_Bag;
        file_ = bag_file;
        break;

        case 2:
        num = ub->Data_type::Sim;
        break;

        case 3:
        num = ub->Data_type::Record;
        file_ = video_file;
        break;

        default:
            system("pkill -f /ros/");
            return 0;
        break;
    }
    ub->data_source(num,file_);

    while(true){
        std::cout<<"请选择操作的功能 (0~7):  \n" 
            << "0 相机内参标定 \n" 
            << "1 摄像头与激光雷达标定 \n" 
            << "2 视觉识别 \n" 
            << "3 视觉车道线检测 \n" 
            << "4 激光雷达检测 \n" 
            << "5 选择相机内参标定文件 \n" 
            << "6 激光雷达与摄像头数据融合效果 \n" 
            << "7 选择相机与激光雷达标定文件(融合距离) \n" 
            << "8 启动算法调参工具\n" 
            << "9 启动rviz" 
            << std::endl;
        std::cin>>cal_is;
        switch (atoi(cal_is.c_str()))
        {
            ub->coordinate_system(1,1,1,1,1,atoi(cal_is.c_str()));
            case 0:
            ub->camera_calibration(8,6,0.6);
            break;
            case 1:
            ub->camera_lidar_calibration();
            break;
            case 2:
            ub->vision_detect(true);
            break;
            case 3:
            ub->vision_lane_follo(true);
            break;
            case 4:
            ub->lidar_detect();
            break;
            case 5:
            ub->select_calibration(0,"/home/ant/work/autoware.ai/catkin-ws/src/startingup_ros/data/Vehicle/sim/calibration/autoware_camera_calibration.yaml");
            break;
            case 6:
            ub->vision_lidar_fusion();
            break;
            case 7:
            ub->select_calibration(1,"/home/ant/work/autoware.ai/catkin-ws/src/startingup_ros/data/Vehicle/sim/calibration/autoware_camera_lidar_calibration.yaml");
            break;
            case 8:
            ub->param_set(true);
            break;
            case 9:
            system("rosrun rviz rviz -d /home/ant/path.rviz &");
            break;

        }
    }
}