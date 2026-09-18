#include "bridge.hpp"

UiBridge::UiBridge()
{
}

UiBridge::~UiBridge()
{

}
void UiBridge::vision_detect(bool on)
{
  std::string cmd;
  if(on){
    cmd = "roslaunch $(rospack find startingup_ros)/data/Vehicle/SensorShelf/custom_launch/yolo3.launch &";
  }else{
    cmd = "pkill -f /visualize_rects /vision_darknet_detect";
  }
  system(cmd.c_str());
}

void UiBridge::vision_lane_follo(bool on)
{
  std::string cmd;
  if(on){
    cmd = "roslaunch $(rospack find startingup_ros)/data/Vehicle/SensorShelf/custom_launch/lane_follo.launch &";
  }else{
    cmd = "pkill -f /lane_follow";
  }
  system(cmd.c_str());
}

bool UiBridge::sensor_status(int type)
{
  switch (type)
  {
    case Sensor_type::lidar:
    break;

    case Sensor_type::camera:
    break;

    default:
    break;
  }
}

void UiBridge::param_set(bool on)
{
  system("pkill -f rqt_reconfigure");
  if(on){
    system("rosrun rqt_reconfigure rqt_reconfigure &");
  }
}

void UiBridge::coordinate_system(double x, double y,double z,double roll,double pitch, double yaw)
{
  system("pkill -f  /base_link_to_localizer");
  std::string cmd = "rosrun tf static_transform_publisher  name:=base_link_to_localizer " ;
  cmd +=  std::to_string(x)+ " "   +  std::to_string(y) + " " +  std::to_string(z)+ " "  +  std::to_string(yaw)+ " "  +  std::to_string(roll)+ " "  +  std::to_string(pitch) + " /base_link " + " /velodyne "+  std::to_string(100);
  system(cmd.c_str());

// std::string start_,end_;
// cmd = "sed  -i 's/";
// cmd += "\"x\".*/\"x\" value=" + x + " \/>";
// cmd += "\"/' $(rospack find startingup_ros)/data/Vehicle/SensorShelf/tf/sensor_tf.launch";
// system(cmd.c_str());
}

void UiBridge::camera_calibration(int row, int col, double spacing)
{
  std::string cmd;
  cmd = "rosrun autoware_camera_lidar_calibrator cameracalibrator.py --square " + std::to_string(spacing) + " --size " +std::to_string(row) +"x"+std::to_string(col)  +  " image:=/image_raw "+  " &";
  system(cmd.c_str());
}
void UiBridge::lidar_detect()
{
 std::string cmd;
  cmd = "roslaunch $(rospack find startingup_ros)/data/Vehicle/SensorShelf/custom_launch/my_lidar_detection.launch  &";
  system(cmd.c_str());
}
void UiBridge::camera_lidar_calibration()
{
 std::string cmd;
  cmd = "rosrun calibration_camera_lidar calibration_toolkit &";
  system(cmd.c_str());
}

void UiBridge::vision_lidar_fusion()
{
 std::string cmd;
 cmd = "roslaunch $(rospack find startingup_ros)/data/Vehicle/SensorShelf/custom_launch/vision_lidar_fusion.launch &";
  system(cmd.c_str());
  
}

void UiBridge::select_calibration(int type,std::string cal_file)
{
  std::string cmd;
  if(type == Calibration_type::Camera){
      // kill_node(-1);
      cmd = sensor_cmd[0] + " file:=" + cal_file + " &";
  }else if(type == Calibration_type::CamAnLidar){
      cmd = "roslaunch $(rospack find startingup_ros)/data/Vehicle/SensorShelf/custom_launch/my_calibration.launch cal_file:=" + cal_file + " &";
  }
  system(cmd.c_str());
}

void  UiBridge::module_node(int type,char* topic_name)
{
  // topic_name = (char* )Module_topic_name[type].c_str();
}

bool  UiBridge::module_status(int type)
{

}

bool  UiBridge::data_source(int type,std::string file)
{
  // kill_node(-1);
  std::string cmd;
  std::string file_="";
  switch (type)
  {
    case Data_type::Real:
    break;

    case Data_type::Sim:
    break;
 
    case Data_type::Ros_Bag:
      file_ = file;
    break;

    case Data_type::Record:
      file_ = file;
    break;

    default:
    break;
  }

  cmd = sensor_cmd[type] + " file:=" + file_ + " &";
  system(cmd.c_str());

}

bool UiBridge::kill_node(int type)
{
    std::string str= "pkill -f ";
#if 1
    if( type == -1){
      str  += "/ros/  " ;
    }else{
      str= str + Module_topic_name[type];
    }
#else
    str  += "/ros/  " ;
#endif
    const char *cmd = str.c_str() ;
  sleep(0.5);
    system(cmd);
}