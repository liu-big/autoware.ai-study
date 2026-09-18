/*
  订阅到数据处理后再发布
 */

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "vehicle/vehicle_interface.hpp"
#include "vehicle/zhongyun.hpp"
#include "vehicle/wuling.hpp"
#include "vehicle/kaiwo.hpp"
#include "vehicle/yunle.hpp"
#include "vehicle/simwuling.hpp"
#include "vehicle/yuhsen.hpp"
#include "vehicle/fantai.hpp"
#include "vehicle/aian.hpp"

// ros 头文件
#include <geometry_msgs/PoseStamped.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_can_msgs/CANPacket.h>
#include <autoware_msgs/VehicleCmd.h>


class VehicleCan
{
public:
  VehicleCan() : nh_(), private_nh_("~")
  {
    private_nh_.param<double>("update_rate", update_rate_, 30);
    private_nh_.param<std::string>("vehicle_name", vehicle_name_, "wuling");

    vehicle_cmd_sub_ = nh_.subscribe("vehicle_cmd", 10, &VehicleCan::vehicleCmdCallback, this);
    can_box_sub_ = nh_.subscribe("can_box_raw", 10, &VehicleCan::vehicleBoxCallback, this);

    current_pose_initialized_ = false;

    if (vehicle_name_.find("wuling") == 0){
      ivehicle = new WulingVehicle();
    }
    else if(vehicle_name_.find("zhongyun") == 0){
      ivehicle = new ZhonyunVehicle();
    }else if(vehicle_name_.find("kaiwo") == 0){
      ivehicle = new KaiWo();
    }else if(vehicle_name_.find("yunle") == 0){
      ivehicle = new YunleVehicle();
    }
    else if(vehicle_name_.find("sim") == 0){
      // ivehicle = new SimWulingVehicle();
      ivehicle = new FantaiVehicle();//仿真台架可用泛太的代码，线控一样，泛太为速度pid控制
    }
    else if(vehicle_name_.find("yuhesen") == 0){
      ivehicle = new YuhsenVehicle();
    }
    else if(vehicle_name_.find("fantai") == 0){
      ivehicle = new FantaiVehicle();
    }
    else if(vehicle_name_.find("aian") == 0){
      // printf("--------埃安底盘---------\n");
      ivehicle = new AianVehicle();
    }
    else{
      ivehicle = new ZhonyunVehicle();
    }

#ifndef AUTO_SEND_TOPIC
      // 车辆底盘需要自动发布,超声波和毫米波无需自动发布
      pthread_t th;
      if (pthread_create(&th, nullptr, &VehicleCan::vehicle_client_cmd, this) != 0)
      {
        std::perror("pthread_create");
        std::exit(1);
      }

      if (pthread_detach(th) != 0)
      {
        std::perror("pthread_detach");
        std::exit(1);
      }
  #endif
  };
  ~VehicleCan(){};

  #ifndef AUTO_SEND_TOPIC
    static void *vehicle_client_cmd(void *arg)
    {
      VehicleCan *_this = (VehicleCan *)arg;
      ros::Rate rate(_this->update_rate_);

      uint8_t recvdata[100];
      while (ros::ok())
      {
        _this->ivehicle->run();
        ros::spinOnce();
        rate.sleep();
      }
    }
#endif
private:
  // ros 相关变量
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber vehicle_cmd_sub_, can_box_sub_;

  // params 获取参数服务器值
  double update_rate_; // replanning and publishing rate [Hz]
  std::string vehicle_name_;

  // classes 组合
  VehicleInferface *ivehicle;          // 必须为指针

  // ros msg variables
  // geometry_msgs::PoseStamped current_pose_local_, current_pose_global_;

  // mist variables
  bool current_pose_initialized_;

  // callback
  void vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr &msg)
  {
    // 数据解析
    publishCanBoxCmd(*msg);
  }
  void vehicleBoxCallback(const autoware_can_msgs::CANPacket::ConstPtr &msg)
  {
    publishCanInfo(*msg);
  }

  // fucntions
  void publishCanInfo(autoware_can_msgs::CANPacket msg_)
  {
    // 数据解析
    autoware_msgs::VehicleCmd msg;
    ivehicle->vehicle_static(msg_);
    // can_info_pub_.publish(msg);
  }
  void publishCanBoxCmd(autoware_msgs::VehicleCmd msg_)
  {
    autoware_can_msgs::CANPacket msg;

    ivehicle->vehicle_cmd(msg_);
    // can_box_pub_.publish(msg);
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "vehicle_can");
  VehicleCan obj;

  ros::spin();
  return 0;
};
