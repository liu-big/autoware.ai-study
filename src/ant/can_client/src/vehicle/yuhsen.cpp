#include "vehicle/yuhsen.hpp"
#include "vehicle/fuzzy_PID.h"

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

static float brake_data = 0, speed_err = 0;
FuzzyPID fuzzypid(1500, 650, 0.3, 0.9, 0.6, 0.219, 16.97, 0.0); //模糊PID初始化

YuhsenVehicle::YuhsenVehicle() : nh_(), private_nh_("~")
{
    can_info_pub_ = nh_.advertise<autoware_can_msgs::CANInfo>("can_info", 1, true);
    can_box_pub_ = nh_.advertise<autoware_can_msgs::CANPacket>("can_box_cmd", 10, true);
    batter_state_pub_ = nh_.advertise<sensor_msgs::BatteryState>("battery_state", 10, true);
    real_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("real_velocity", 10, true);
    current_velocity_sub_ = nh_.subscribe("/current_velocity", 10, &YuhsenVehicle::callbackCurrentVelocity, this);
    vehicle_raw_sub_ = nh_.subscribe("/vehicle_raw_cmd", 10, &YuhsenVehicle::callbackVehicleRaw, this);

    sub_final_waypoints_ = nh_.subscribe("/final_waypoints", 1, &YuhsenVehicle::callbackFinalWaypoints, this);


    sub_Trajectory_Cost = nh_.subscribe("/local_trajectory_cost", 1, &YuhsenVehicle::callbackGetLocalTrajectoryCost, this);
    private_nh_.param<double>("speed_gain", speed_gain, 1.6);

    spd_ratio_sub = nh_.subscribe("safe_range_ctrl", 1, &YuhsenVehicle::callbackSpeedRatio, this);
    sub_GoalRemainingDistance = nh_.subscribe("/goal_remaining_distance", 1, &YuhsenVehicle::callbackGetGoalRemainingDistance, this);
}

YuhsenVehicle::~YuhsenVehicle()
{
}

void YuhsenVehicle::callbackFinalWaypoints(const autoware_msgs::Lane &final_waypoints)
{
    // m_nTargetNum = final_waypoints.waypoints.size();
}

void YuhsenVehicle::callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr& msg)
{
  // m_TrajectoryBestCost.bBlocked = msg->is_blocked;
  // m_TrajectoryBestCost.index = msg->lane_index;
  // m_TrajectoryBestCost.closest_obj_distance = msg->closest_object_distance;
  // m_TrajectoryBestCost.closest_obj_velocity = msg->closest_object_velocity;
  // back_distance = msg->closest_object_distance;
    // follow_on = false;
    // static int count_ = 0;
    // if (msg->is_blocked && msg->closest_object_distance < (maxdistancetoavoid-1.0))
    // {
    //         follow_on = true;
    // }
}

void YuhsenVehicle::callbackGetGoalRemainingDistance(const std_msgs::Float32ConstPtr &msg)
{
    // m_fRemainingDistance = msg->data;
}

void YuhsenVehicle::callbackSpeedRatio(const autoware_msgs::Vehicle_Ctrl_StatusConstPtr& msg)
{
    speed_ratio = msg->safe_ctrl;
    m_aSafeDir[0] = msg->safe_from;
    m_aSafeDir[1] = msg->safe_lear;
}


void YuhsenVehicle::fuzzy_PIDINT()
{
    int deltaKpMatrix[7][7]={{PB,PB,PM,PM,PS,ZO,ZO},
                        {PB,PB,PM,PS,PS,ZO,NS},
                        {PM,PM,PM,PS,ZO,NS,NS},
                        {PM,PM,PS,ZO,NS,NM,NM},
                        {PS,PS,ZO,NS,NS,NM,NM},
                        {PS,ZO,NS,NM,NM,NM,NB},
                        {ZO,ZO,NM,NM,NM,NB,NB}};
	int deltaKiMatrix[7][7]={{NB,NB,NM,NM,NS,ZO,ZO},
                        {NB,NB,NM,NS,NS,ZO,ZO},
                        {NB,NM,NS,NS,ZO,PS,PS},
                        {NM,NM,NS,ZO,PS,PM,PM},
                        {NM,NS,ZO,PS,PS,PM,PB},
                        {ZO,ZO,PS,PS,PM,PB,PB},
                        {ZO,ZO,PS,PM,PM,PB,PB}};
	int deltaKdMatrix[7][7]={{PS,NS,NB,NB,NB,NM,PS},
                        {PS,NS,NB,NM,NM,NS,ZO},
                        {ZO,NS,NM,NM,NS,NS,ZO},
                        {ZO,NS,NS,NS,NS,NS,ZO},
                        {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
                        {PB,NS,PS,PS,PS,PS,PB},
                        {PB,PM,PM,PM,PS,PS,PB}};
	float e_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float de_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float Kp_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float Ki_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float Kd_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    // FuzzyPID fuzzypid(1500,650,0.3,0.9,0.6,0.01,0.04,0.01);
	fuzzypid.setMf("trimf",e_mf_paras,"trimf",de_mf_paras,"trimf",Kp_mf_paras,"trimf",Ki_mf_paras,"trimf",Kd_mf_paras);
	fuzzypid.setRuleMatrix(deltaKpMatrix,deltaKiMatrix,deltaKdMatrix);

}

float YuhsenVehicle::Speed_FilterOut(float target)
{
	static float target_last = 0,target_out,target_2;
	static bool flag = true;

	target_last = target_2;
	target_2 = target;

	if(target!=0)
	{
		if(flag)
		{
			if(target_2-target_last>0.5){
				target_out = target_last;
				flag = false;
			}
			else
				target_out = target_2;
		}
		else
		{
			if(target_out<target_2)
			{
				target_out+=0.04;
			}
			else{
				target_out = target_2;
				flag = true;
			}
		}
	}
	else
		target_out = 0;
	return  target_out;
}

void YuhsenVehicle::vehicle_cmd(autoware_msgs::VehicleCmd msg)
{
    autoware_msgs::VehicleCmd _m;
    static int16_t count=0;//心跳信号
    count++;
    if(count>65535)
        count = 0;

    _m.gear_cmd.gear = GearShift::Drive;  //设置前进档位
    _m.twist_cmd.twist.linear.x = msg.twist_cmd.twist.linear.x*speed_gain;//速度
    _m.steer_cmd.steer = msg.ctrl_cmd.steering_angle *( 180 /3.1415);// 弧度 转为度

    if(speed_ratio == 0 ){  //是否有障碍物在安全距离内
        _m.twist_cmd.twist.linear.x =  0;
        _m.brake_cmd.brake = 1;
    }else{
        _m.brake_cmd.brake = 0;
    }

    // _m.twist_cmd.twist.linear.x = _m.twist_cmd.twist.linear.x*;
    if(remote_mode != 0)
    {
        double objet_speed = Speed_FilterOut(_m.twist_cmd.twist.linear.x);
        ctrl_cmd_Vehicle(_m.gear_cmd.gear, objet_speed, _m.steer_cmd.steer, _m.brake_cmd.brake, count);// 运动控制指令
//    ctrl_cmd_Peripheral(uint8_t io_en, uint8_t headlamp_en, uint8_t turn_en, uint8_t speaker_en, count);// 外设控制指令

        ROS_INFO("---- [can send] gear: %d, speed: %f , angle: %d, brake: %d ", _m.gear_cmd.gear, objet_speed, _m.steer_cmd.steer, _m.brake_cmd.brake);
        ROS_INFO("**** [can raw ] gear: %d, speed: %f , angle: %f, brake: %d , corn_lamp: %d, headlamp: %d", can_info_.driveshift, can_info_.speed, can_info_.angle, 
            can_info_.brakepedal, can_info_.shiftfrmprius, can_info_.light);
    }
}

//远程驾驶
void YuhsenVehicle::callbackVehicleRaw(const autoware_remove_msgs::VehicleCmd& msg)
{
    autoware_msgs::VehicleCmd _remote;
    static int16_t count=0;//心跳信号
    count++;
    if(count>65535)
        count = 0;

    if(msg.mode==0)
    {
        remote_mode = 0;//退出自动驾驶

        _remote.twist_cmd.twist.linear.x = 20.0*(msg.acc/100.0);
        _remote.steer_cmd.steer = msg.angle/21.6;
        if(msg.brake>0)
            _remote.brake_cmd.brake = 1;
        else
            _remote.brake_cmd.brake = 0;

//     corn_lamp_value = msg.corn_lamp;
//     headlamp_value = msg.headlamp;

        switch (msg.gear)
        {
            case 0:
                _remote.gear_cmd.gear = GearShift::Neutral;
                break;
            case 1:
                _remote.gear_cmd.gear = GearShift::Drive;
                break;
            case 2:
                _remote.gear_cmd.gear = GearShift::Reverse;
                break;
            case 3:
                _remote.gear_cmd.gear = GearShift::Parking;
                break;
            default:
                _remote.gear_cmd.gear = GearShift::Neutral;
                break;
        }
        ctrl_cmd_Vehicle(_remote.gear_cmd.gear, _remote.accel_cmd.accel, _remote.steer_cmd.steer, _remote.brake_cmd.brake, count);// 运动控制指令
        ctrl_cmd_Peripheral(1, msg.headlamp, msg.corn_lamp, 0, count);// 外设控制指令
        ROS_INFO("---- [can send] gear: %d, speed: %f , angle: %d, brake: %d , corn_lamp: %d, headlamp: %d", _remote.gear_cmd.gear, _remote.twist_cmd.twist.linear.x,
            _remote.steer_cmd.steer, _remote.brake_cmd.brake, msg.corn_lamp, msg.headlamp);
        ROS_INFO("**** [can raw ] gear: %d, speed: %f , angle: %f, brake: %d , corn_lamp: %d, headlamp: %d", can_info_.driveshift, can_info_.speed, can_info_.angle, 
            can_info_.brakepedal, can_info_.shiftfrmprius, can_info_.light);
    }else
        remote_mode = 1;
}

void YuhsenVehicle::vehicle_static(autoware_can_msgs::CANPacket msg)
{
    uint8_t data[8];//= msg.dat;
    uint8_t dlc_;
    memcpy(data, &msg.dat, sizeof(msg.dat));

    static sensor_msgs::BatteryState bat_sta;//ROS话题发布电池状态

    if(data[7] == xor_check(data))//异或校验
    {
        if (msg.id == ctrl_fb_CANID)//车辆状态
        {
            ctrl_fb_t re_msg;
            Unpack_ctrl_fb_chassis_fr_can_protoc(&re_msg, data, dlc_);

            can_info_.driveshift = re_msg.ctrl_fb_gear;    //当前档位
            can_info_.speed = ctrl_fb_ctrl_fb_velocity_fromS(re_msg.ctrl_fb_velocity)*3600.0/1000.0;//速度精度0.001m/s/bit   当前车速,转换成千米/小时
            can_info_.angle = ctrl_fb_ctrl_fb_steering_fromS(re_msg.ctrl_fb_steering);//角度精度0.01°/bit  /方向盘角度
            can_info_.brakepedal = re_msg.ctrl_fb_Brake;//制动状态
            can_info_.devmode = re_msg.ctrl_fb_mode;//驾驶模式
        }
        else if (msg.id == io_fb_CANID)//外设状态
        {
            io_fb_t re_msg;
            Unpack_io_fb_chassis_fr_can_protoc(&re_msg, data, dlc_);
            re_msg.io_fb_enable;//I/O 控制使能状态反馈
            can_info_.light = re_msg.io_fb_upper_beam_headlamp;//远光灯开关状态反馈
            can_info_.shiftfrmprius = re_msg.io_fb_turn_lamp;//转向灯开关状态反反馈
            re_msg.io_fb_braking_lamp;//制动灯开关状态反馈
            re_msg.io_fb_speaker;//扬声器开关状态反馈
            re_msg.io_fb_fm_impact_sensor;//前中防撞条开关状态反馈
            re_msg.io_fb_rm_impact_sensor;//后中防撞条开关状态反馈

            // ROS_ERROR("%d  %d  %d  %d  %d  %d  %d",re_msg.io_fb_enable,  re_msg.io_fb_upper_beam_headlamp, re_msg.io_fb_turn_lamp, re_msg.io_fb_braking_lamp,
            //         re_msg.io_fb_speaker, re_msg.io_fb_fm_impact_sensor, re_msg.io_fb_rm_impact_sensor);
        }
        else if (msg.id == bms_Infor_CANID)//电池状态
        {
            bms_Infor_t re_msg;
            Unpack_bms_Infor_chassis_fr_can_protoc(&re_msg, data, dlc_);
            bat_sta.voltage = re_msg.bms_Infor_voltage*0.01;//当前电池电压
            bat_sta.current = re_msg.bms_Infor_current*0.01;//当前电池电流
            re_msg.bms_Infor_remaining_capacity;//当前电池剩余容量
        }
        else if (msg.id == bms_flag_Infor_CANID)//电池状态
        {
            bms_flag_Infor_t re_msg;
            Unpack_bms_flag_Infor_chassis_fr_can_protoc(&re_msg, data, dlc_);
            bat_sta.percentage = re_msg.bms_flag_Infor_soc;//当前剩余电量百分比
        }
        bat_sta.header.stamp = ros::Time::now();
        can_info_.battery = bat_sta.percentage;
        can_info_.voltage = bat_sta.voltage;
        batter_state_pub_.publish(bat_sta);//发布电池状态话题
    }


    // ROS_ERROR("%d  %d  %d  %d  %d",h_msg.hardware_input_ignition,h_msg.hardware_input_KeyOn,h_msg.hardware_input_D,h_msg.hardware_input_N,h_msg.hardware_input_R);
    // ROS_ERROR("%d  %d  %d  %d  %d",h_msg.hardware_input_Acc,h_msg.hardware_input_Dec,h_msg.hardware_Foot_brake_input,h_msg.hardware_input_Analog_Weight_V,h_msg.HarwareGearCmd);
    // ROS_ERROR("%d  %d  %d  %d  %d",h_msg.DrvModFlag,h_msg.gearcmd,h_msg.GW_Target_Speed,h_msg.SCU_Target_Speed,h_msg.HarwareTargetSpd);

}

////////////////////// SET ////////////////////////////
// 运动控制指令
void YuhsenVehicle::ctrl_cmd_Vehicle(uint8_t gear, float speed, float steering, int16_t brake, int16_t count)
{
    uint8_t len;
    uint8_t ide;
    uint8_t data[8];

    ctrl_cmd_t msg;

    msg.ctrl_cmd_gear = gear;//档位
    // ROS_ERROR("-------------------  %d", (int)(speed/3600.0*1000/0.001));
    msg.ctrl_cmd_velocity = (int)(speed/3600.0*1000/0.001);//速度转化0.001m/s
    msg.ctrl_cmd_steering = (int)(steering/0.01);//角度
    msg.ctrl_cmd_Brake = brake;//制动
    msg.ctrl_cmd_alive_cnt = count;//心跳信号
    Pack_ctrl_cmd_chassis_fr_can_protoc(&msg, data, &len, &ide);
    data[7] = xor_check(data);//消息异或校验

    autoware_can_msgs::CANPacket canpack;
    canpack.id = (int)ctrl_cmd_CANID;
    canpack.len = sizeof(data);
    memcpy(&canpack.dat, data, sizeof(data));
    can_box_pub_.publish(canpack);
}
// 外设控制指令
void YuhsenVehicle::ctrl_cmd_Peripheral(uint8_t io_en, uint8_t headlamp_en, uint8_t turn_en, uint8_t speaker_en, uint8_t count)
{
    uint8_t len;
    uint8_t ide;
    uint8_t data[8];

    io_cmd_t msg;

    msg.io_cmd_enable = io_en;//  I/O 控制使能
    msg.io_cmd_upper_beam_headlamp = headlamp_en;//远光灯开关
    msg.io_cmd_turn_lamp = turn_en;//转向灯开关
    msg.io_cmd_speaker = speaker_en;//扬声器开关
    msg.io_cmd_alive_cnt = count;//心跳信号
    // msg.io_cmd_check_bcc = xor_check(data);//消息异或校验
    Pack_io_cmd_chassis_fr_can_protoc(&msg, data, &len, &ide);
    data[7] = xor_check(data);//消息异或校验

    autoware_can_msgs::CANPacket canpack;
    canpack.id = (int)io_cmd_CANID;
    canpack.len = sizeof(data);
    memcpy(&canpack.dat, data, sizeof(data));
    can_box_pub_.publish(canpack);
}

void YuhsenVehicle::run()
{
    can_info_pub_.publish(can_info_);
}

void YuhsenVehicle::callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg)
{
  //can_info_.speed = msg->twist.linear.x * 3.6;   // km/h
}

//异或校验
uint16_t YuhsenVehicle::xor_check(void* canFrame) //异或校验函数
{
    uint8_t sum = 0;
    uint8_t *ptr = (uint8_t *)canFrame;
    for (int i = 0; i < 7; i++)
    {
        sum ^= ptr[i];
    }
    return sum;
}
