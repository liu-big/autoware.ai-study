#ifndef _YUHSEN_H_
#define _YUHSEN_H_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "vehicle/vehicle_interface.hpp"
#include "vehicle/yuhsen_lib/canbase.hpp"
// ros 头文件
#include <autoware_msgs/Gear.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/Lane.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_can_msgs/CANPacket.h>
#include <autoware_remove_msgs/VehicleCmd.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/BatteryState.h>
#include <autoware_msgs/Vehicle_Ctrl_Status.h>

class YuhsenVehicle : public VehicleInferface
{
public:
    YuhsenVehicle();
    ~YuhsenVehicle();


    void vehicle_cmd(autoware_msgs::VehicleCmd msg);
    void vehicle_static(autoware_can_msgs::CANPacket msg);
    void run();

private:
    // ros 相关变量
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher can_info_pub_, can_box_pub_, real_velocity_pub_;
    ros::Publisher batter_state_pub_;
    ros::Subscriber current_velocity_sub_, spd_ratio_sub, sub_final_waypoints_, sub_Trajectory_Cost, sub_GoalRemainingDistance, vehicle_raw_sub_;

    autoware_can_msgs::CANInfo can_info_;
    double speed_ratio = 1;
    bool m_aSafeDir[2];
    double speed_gain;


    uint8_t remote_mode = 1;

    void callbackSpeedRatio(const autoware_msgs::Vehicle_Ctrl_StatusConstPtr &msg);
    void callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
    void callbackFinalWaypoints(const autoware_msgs::Lane &final_waypoints);
    void callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr &msg);
    void callbackGetGoalRemainingDistance(const std_msgs::Float32ConstPtr &msg);
    //远程驾驶
    void callbackVehicleRaw(const autoware_remove_msgs::VehicleCmd& msg);
    uint16_t xor_check(void* canFrame); //异或校验函数

    void fuzzy_PIDINT();
    // double pid_break(double obj_spd, double current_spd);
    float Speed_FilterOut(float target);
    // bool node_state(void);//节点状态判断函数

    ros::V_string node_ultrasound;


    enum GearShift
    {
        Drive = 4,
        Neutral = 3,
        Reverse = 2,
        Parking = 1,
    };

    // 运动控制指令
    void ctrl_cmd_Vehicle(uint8_t gear, float speed, float steering, int16_t brake, int16_t count);
    // 外设控制指令
    void ctrl_cmd_Peripheral(uint8_t io_en, uint8_t headlamp_en, uint8_t turn_en, uint8_t speaker_en, uint8_t count);

};

#endif