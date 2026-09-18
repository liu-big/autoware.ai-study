#include "vehicle/yunle.hpp"
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

YunleVehicle::YunleVehicle() : nh_(), private_nh_("~")
{
  can_info_pub_ = nh_.advertise<autoware_can_msgs::CANInfo>("can_info", 1, true);
  can_box_pub_ = nh_.advertise<autoware_can_msgs::CANPacket>("can_box_cmd", 10, true);
  batter_state_pub_ = nh_.advertise<sensor_msgs::BatteryState>("battery_state", 10, true);
  real_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("real_velocity", 10, true);
  current_velocity_sub_ = nh_.subscribe("/current_velocity", 10, &YunleVehicle::callbackCurrentVelocity, this);

  sub_final_waypoints_ = nh_.subscribe("/final_waypoints", 1, &YunleVehicle::callbackFinalWaypoints, this);


  sub_Trajectory_Cost = nh_.subscribe("/local_trajectory_cost", 1, &YunleVehicle::callbackGetLocalTrajectoryCost, this);

  private_nh_.param<double>("/op_common_params/maxSteerAngle", maxsteerangle, 0.47124);
  private_nh_.param<double>("/op_common_params/maxDistanceToAvoid", maxdistancetoavoid, 5);
  private_nh_.param<double>("add_speed_value", add_speed_value, 3);
  private_nh_.param<double>("speed_gain", speed_gain, 1.6);
  private_nh_.param<double>("base_brake_value", base_brake_value, 35);
  private_nh_.param<bool>("bench_debug", bench_debug_value, false);

  private_nh_.param<double>("stop_brake_increase", stop_brake_increase, 0.1);
  private_nh_.param<double>("normal_brake_increase", normal_brake_increase, 0.8);
  private_nh_.param<bool>("enable_recover", enable_recover, false);

  spd_ratio_sub = nh_.subscribe("safe_range_ctrl", 1, &YunleVehicle::callbackSpeedRatio, this);
  sub_GoalRemainingDistance = nh_.subscribe("/goal_remaining_distance", 1, &YunleVehicle::callbackGetGoalRemainingDistance, this);
  speed_ratio = 1.0;
  fuzzy_PIDINT();

  ctrl_mode = CtrlMode::Normal;
}

YunleVehicle::~YunleVehicle()
{
}

void YunleVehicle::callbackFinalWaypoints(const autoware_msgs::Lane &final_waypoints)
{
    m_nTargetNum = final_waypoints.waypoints.size();
}

void YunleVehicle::callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr& msg)
{
  // m_TrajectoryBestCost.bBlocked = msg->is_blocked;
  // m_TrajectoryBestCost.index = msg->lane_index;
  // m_TrajectoryBestCost.closest_obj_distance = msg->closest_object_distance;
  // m_TrajectoryBestCost.closest_obj_velocity = msg->closest_object_velocity;
  // back_distance = msg->closest_object_distance;
  follow_on = false;
  static int count_ = 0;
  if (msg->is_blocked && msg->closest_object_distance < (maxdistancetoavoid-1.0))
  {
          follow_on = true;
  }
}

void YunleVehicle::callbackGetGoalRemainingDistance(const std_msgs::Float32ConstPtr &msg)
{
    m_fRemainingDistance = msg->data;
}

void YunleVehicle::callbackSpeedRatio(const autoware_msgs::Vehicle_Ctrl_StatusConstPtr& msg)
{
    // ctrl_type
    static int recover_count = 0;
    static int forward_count = 0;
    static int follow_count = 0;
    static int normal_count = 0;

    speed_ratio = msg->safe_ctrl;

    if(enable_recover == false){
        return;
    }

    m_aSafeDir[0] = (bool)msg->safe_from;
    m_aSafeDir[1] = (bool)msg->safe_lear;

    if(m_fRemainingDistance < 10.0)
    {
        forward_count = 150;
        gear_reverse_on = false;
        back_distance = 0.0;
        recover_count = 0;
        return;
    }

    // if(follow_on){
    //     if(++recover_count >= 60){
    //         recover_count = 60;
    //         gear_reverse_on = true;
    //         speed_ratio = 1;
    //     }
    //     ROS_ERROR("1 recover_count : %d,gear_reverse_on: %d",recover_count,gear_reverse_on);

    // }

    ctrl_mode = CtrlMode::Normal;
    if ((m_aSafeDir[0] == true && m_aSafeDir[1] == false)){
        ctrl_mode = CtrlMode::Recover;
    }else if (m_aSafeDir[0] == false && m_aSafeDir[1] == true){
        // if (ctrl_mode == CtrlMode::Recover && ctrl_mode == CtrlMode::FollowingDistance){
        // }
        ctrl_mode = CtrlMode::Forward;
    }else if (m_aSafeDir[0] == true && m_aSafeDir[1] == true){
        ctrl_mode = CtrlMode::Normal;
    }else{
        if (back_distance > 0 && back_distance < 2.5)
        {
            ctrl_mode = CtrlMode::Recover;
        }else if(follow_on){
            ctrl_mode = CtrlMode::FollowingDistance;
        }
        if(  ctrl_mode == CtrlMode::Forward){

            if(++normal_count < 30){
                ctrl_mode == CtrlMode::Forward;
            }
        }


    }


    if(ctrl_mode == CtrlMode::Recover){
        if (++recover_count >= 30)
        {
            gear_reverse_on = true;
            backward_on = true;
            speed_ratio = 1;
        }
    }else if(ctrl_mode == CtrlMode::FollowingDistance){

        if(++follow_count >= 30){
            gear_reverse_on = true;
            speed_ratio = 1;
        }
    }
    else if (ctrl_mode == CtrlMode::Forward)
    {
        if (++forward_count > 30)
        {
            speed_ratio = 1;
        }
        gear_reverse_on = false;
    }
    else if (ctrl_mode == CtrlMode::Normal)
    {
        recover_count = 0;
        forward_count = 0;
        follow_count = 0;
        back_distance = 0;
        normal_count = 0;
        backward_on = false;
        gear_reverse_on = false;
    }
}

void YunleVehicle::fuzzy_PIDINT()
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

double YunleVehicle::pid_break(double obj_spd, double current_spd)
{
  speed_err=obj_spd-current_spd;

if(obj_spd != 0)
{
  if(speed_err<0)
  {
    brake_data+=normal_brake_increase;
  }
  else
    brake_data = 0;

}
else
      brake_data+=normal_brake_increase*2.0;
      // ROS_INFO("********************%f      %f    \n ",speed_err,brake_data);
    return 0;
}

float YunleVehicle::Speed_FilterOut(float target)
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
				target_out+=0.02;
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

bool YunleVehicle::node_state(void)//重要节点状态判断函数
{
    ros::V_string node_if;
    ros::master::getNodes(node_if);

    auto it_if = find(node_if.begin(),node_if.end(),"/lslidar_c16_decoder_node");
    if(it_if==node_if.end())
    {
      ROS_ERROR("NO find /lslidar_c16_decoder_node!!!!!");
      return false;
    }
    auto it_if2 = find(node_if.begin(),node_if.end(),"/ultra_bf19");
    if(it_if2==node_if.end())
    {
      ROS_ERROR("NO find /ultra_bf19!!!!!");
      return false;
    }
    return true;
}


void YunleVehicle::vehicle_cmd(autoware_msgs::VehicleCmd msg)
{
    autoware_msgs::VehicleCmd _m;

    _m.mode = dev_mode;

    _m.gear_cmd.gear = GearShift::Drive;  //设置前进档位

    static float pid_data,current_speed;
    if(can_info_.speed<0)
        can_info_.speed =0;
    current_speed = can_info_.speed;
    double _obj_speed = msg.twist_cmd.twist.linear.x*speed_gain;

    static int stop_brake_=0;
    float target;
    // if(can_info_.devmode && can_info_.driveshift != 1){
    // if(_m.mode){
     if(1){
        // float pid_data = pid_ctrl(msg.twist_cmd.twist.linear.x,current_speed);//
        if(_obj_speed <3&& _obj_speed !=0)//限制转弯速度过小
        _obj_speed = 3;

        // _obj_speed = _obj_speed*std::min(speed_slow[0],speed_slow[2]);//目标速度*障碍物减速百分比（前和前侧面的最小减速比）
        target = Speed_FilterOut(_obj_speed);//目标速度平滑处理
        // ROS_ERROR("Test_Data1------    %f      ---%f    ---%f",_obj_speed,current_speed,target);

        if(speed_ratio == 0 || !node_state()){  //是否有障碍物在安全距离内判断重要节点是否存在  || !node_state()
            target =  0;
            stop_brake_ = 1;
        }else{
            stop_brake_ = 0;
        }

        if(target == 0){
            pid_data = 0;
            fuzzypid.init();
        }else{
            // pid_data = fuzzypid.realize(target,current_speed);//模糊PID处理数据
        }
        // pid_break(_obj_speed,current_speed);

       
        // _m.accel_cmd.accel = pid_data >30?30:pid_data;
        _m.accel_cmd.accel = target;
        if(_m.accel_cmd.accel<0)
            _m.accel_cmd.accel  = 0;
    }else{
        _m.accel_cmd.accel = 0;
        Speed_FilterOut(0);//确保退出自动驾驶时速度从零开始计算
        // fuzzypid.showInfo();
    }
     
    _m.brake_cmd.brake = (int)brake_data;
    ROS_ERROR("Test_Data1------    %f      ---%f    ---%f",_obj_speed,current_speed,target);
    // if(_m.mode)
    _m.mode = 1;
    sendScuData(_m.mode, _m.gear_cmd.gear, target, msg.ctrl_cmd.steering_angle, stop_brake_);

    // msg.ctrl_cmd.steering_angle = -120*msg.ctrl_cmd.steering_angle/maxsteerangle*10;          //  转向角度


    ROS_INFO("------------ [can send] mode: %d, acc: %d, brake: %d, angle: %f, gear: %d, speed:%f",_m.mode,_m.accel_cmd.accel,stop_brake_,msg.ctrl_cmd.steering_angle,_m.gear_cmd.gear,target);
    ROS_WARN ("------------ [can raw] mode: %d, acc: %d, brake: %d, angle: %f, gear: %d, speed:%f",can_info_.devmode,can_info_.drivepedal,can_info_.brakepedal,can_info_.angle,can_info_.driveshift,can_info_.speed);
}

void YunleVehicle::vehicle_static(autoware_can_msgs::CANPacket msg)
{
    uint8_t data[8];//= msg.dat;
    uint8_t dlc_;
    CCU_Status_t re_msg;
    memcpy(data, &msg.dat, sizeof(msg.dat));

    if (msg.id == CCU_Status_CANID)
    {
        Unpack_CCU_Status__YunleCAN(&re_msg, data, dlc_);

        can_info_.speed = float(re_msg.CCU_Vehicle_Speed/10.0)*3600.0/1000.0;   //当前车速,转换成千米/小时
        can_info_.driveshift = re_msg.CCU_ShiftLevel_Sts;    //当前档位
        can_info_.brakepedal = re_msg.CCU_P_Sts;      //P档状态
        if(re_msg.Steering_Wheel_Direction)   //实际方向盘方向
            can_info_.angle = re_msg.CCU_Steering_Wheel_Angle*27.0/1200.0;  //转换成角度最大+-27度
        else
            can_info_.angle = -re_msg.CCU_Steering_Wheel_Angle*27.0/1200.0;  //转换成角度最大+-27度
        can_info_.devmode = re_msg.CCU_Drive_Mode;    //当前驾驶模式
        // ROS_ERROR("raw     %d  %d  %d",re_msg.CCU_ShiftLevel_Sts,re_msg.CCU_P_Sts,re_msg.CCU_Ignition_Sts);
    }
    else if(msg.id == BMS_SOC_CANID)//解析电池状态
            getBms_fbk(data);
    else if(msg.id == Hardware_inpute_CANID)//解析遥控器状态
    {
        Hardware_inpute_t h_msg;

        Unpack_Hardware_inpute__YunleCAN(&h_msg, data, dlc_);
        if(dev_mode == 1){
            if(h_msg.hardware_input_D | h_msg.hardware_input_N | h_msg.hardware_input_Acc | h_msg.hardware_input_Dec | h_msg.hardware_Foot_brake_input)
                dev_mode = 0;
        }
        else{
            static uint8_t last_brake = 0;
            if(last_brake - h_msg.hardware_Foot_brake_input >0)
                dev_mode = 1;
            if(last_brake!=h_msg.hardware_Foot_brake_input)
            {
                last_brake = h_msg.hardware_Foot_brake_input;
            }

        }
        // ROS_INFO("**************************");
        // ROS_ERROR("%d  %d  %d  %d  %d",h_msg.hardware_input_ignition,h_msg.hardware_input_KeyOn,h_msg.hardware_input_D,h_msg.hardware_input_N,h_msg.hardware_input_R);
        // ROS_ERROR("%d  %d  %d  %d  %d",h_msg.hardware_input_Acc,h_msg.hardware_input_Dec,h_msg.hardware_Foot_brake_input,h_msg.hardware_input_Analog_Weight_V,h_msg.HarwareGearCmd);
        // ROS_ERROR("%d  %d  %d  %d  %d",h_msg.DrvModFlag,h_msg.gearcmd,h_msg.GW_Target_Speed,h_msg.SCU_Target_Speed,h_msg.HarwareTargetSpd);
        // ROS_INFO("**************************");
    }

    if(bench_debug_value){    //台架调试（底盘悬空）需要加入小PID指令
        autoware_can_msgs::CANPacket canpack;
        uint8_t data1[8] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        uint8_t data2[8] = {0x00, 0x00, 0x00, 0x00, 0x28, 0x10, 0x00, 0x00};
        static bool flag = true;
        if(flag)
        {
            canpack.id = (int)0x710;
            canpack.len = sizeof(data1);
            memcpy(&canpack.dat, data1, sizeof(data1));
            can_box_pub_.publish(canpack);
        }else{
            canpack.id = (int)0x715;
            canpack.len = sizeof(data2);
            memcpy(&canpack.dat, data2, sizeof(data2));
            can_box_pub_.publish(canpack);
        }
        flag = !flag;
    }

}

////////////////////// SET ////////////////////////////
// 设置SCU数据
void YunleVehicle::sendScuData(uint8_t Drive_Mode, uint8_t shiftLevel, float target, float steering, int16_t brake)
{
    uint8_t len;
    uint8_t ide;
    uint8_t data[8];

    SCU_t msg;

    // msg.SCU_ShiftLevel_Req = shiftLevel;               //      Bits=02.  [ 0     , 3      ]  Unit:''      档位
    msg.SCU_ACC_Mode = 0;                     //      Bits=02.  [ 0     , 3      ]  Unit:''         刹车模式
    msg.SCU_Brake_Mode = 2;                   //  刹车模式  设置成线性刹车
    msg.SCU_Drive_Mode_Req = Drive_Mode;               //        模式标志位
    // msg.SCU_Steering_Wheel_Angle = 120*steering/maxsteerangle*10;         //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:''      Factor= 0.1
    // msg.SCU_Target_Speed = target;   //将弧度转为度再分成最大120  精度0.1           //      Bits=09.  [ 0     , 51     ]  Unit:'km/h'  Factor= 0.1
    msg.SCU_ShiftLevel_Req = 1;
    msg.SCU_Steering_Wheel_Angle = -120*steering/maxsteerangle*10;          //  转向角度
    msg.SCU_Target_Speed = target*10;//目标速度
    msg.SCU_Brk_En = brake;                       //刹车使能
    // msg.SCU_Brake_Coefficient = 10;

    msg.GW_Left_Turn_Light_Req = 0;           //      Bits=02.  [ 0     , 3      ]  Unit:''
    msg.GW_Right_Turn_Light_Req = 0;          //      Bits=02.  [ 0     , 3      ]  Unit:''
    msg.GW_Hazard_Light_Req = 0;              //      Bits=02.  [ 0     , 3      ]  Unit:''
    msg.GW_Position_Light_Req = 0;            //      Bits=02.  [ 0     , 3      ]  Unit:''
    msg.GW_LowBeam_Req = 0;                   //      Bits=02.  [ 0     , 3      ]  Unit:''
    msg.GW_HighBeam_Req = 0;                  //      Bits=02.  [ 0     , 3      ]  Unit:''
    msg.GW_RearFogLight_Req = 0;              //      Bits=02.  [ 0     , 3      ]  Unit:''
    msg.GW_Horn_Req = 0;                      //      Bit
    msg.SCU_Brake_Coefficient = 0;
    Pack_SCU__YunleCAN(&msg, data, &len, &ide);

    autoware_can_msgs::CANPacket canpack;
    canpack.id = (int)SCU_CANID;
    canpack.len = sizeof(data);
    memcpy(&canpack.dat, data, sizeof(data));
    can_box_pub_.publish(canpack);
}


// 设置驾驶模式
bool YunleVehicle::setMode(int32_t en)
{
//   autoware_can_msgs::CANPacket msg;

}

// 设置驱动百分比
void YunleVehicle::setAccPercent(int32_t value, bool en)
{
    uint8_t len;
    uint8_t ide;
    uint8_t data[8];

    SCU_t msg;
    
    Pack_SCU__YunleCAN(&msg, data, &len, &ide);

    autoware_can_msgs::CANPacket canpack;
    canpack.id = (int)SCU_CANID;
    canpack.len = sizeof(data);
    memcpy(&canpack.dat, data, sizeof(data));
    can_box_pub_.publish(canpack);
}
void YunleVehicle::setBrakePercent(int32_t value, bool en)
{

//   uint8_t len;
//   uint8_t ide;
//   uint8_t data[8];
//   brake_control_t msg;
//   msg.Brake_Enable_control = en;
//   value = (value / 0.001525);
//   if (value > 65535)
//     value = 65535;
//   else if (value < 0)
//     value = 0;

//   msg.Brake_Tq_Req = (uint16_t)value;

//   Pack_brake_control_CAN_yeyazhidong_20210(&msg, data, &len, &ide);

//   autoware_can_msgs::CANPacket canpack;
//   canpack.id = (int)VehicleDataType::Brake_Cmd;
//   canpack.len = sizeof(data);
//   memcpy(&canpack.dat, data, sizeof(data));
//   can_box_pub_.publish(canpack);
}

// 设置制动百分比
void YunleVehicle::setSteerPostion(int32_t value, bool en)
{

//   value = (((value / 540.0 * (maxsteerangle * ( 180 /3.1415))) + (maxsteerangle * ( 180 /3.1415))) / 0.0008545);
//   if (value > 65535)
//     value = 65535;
//   else if (value < 0)
//     value = 0;

//   uint8_t len;
//   uint8_t ide;
//   uint8_t data[8];
//   steering_control_t msg;
//   msg.Steering_Enable_control = en;
//   //  通过can协议移位得到data[8]数据
//   msg.Steering_Pos_Req = value;

//   Pack_steering_control_CAN_yeyazhidong_20210(&msg, data, &len, &ide);

//   // 封装can数据发布
//   autoware_can_msgs::CANPacket canpack;
//   canpack.id = (int)VehicleDataType::Steer_Cmd;
//   canpack.len = sizeof(data);
//   memcpy(&canpack.dat, data, sizeof(data));
//   can_box_pub_.publish(canpack);
}

// 设置档位
void YunleVehicle::setGear(uint8_t valude, bool en)
{
//   uint8_t len;
//   uint8_t ide;
//   uint8_t data[8];
//   Gear_Shift_Cmd_t msg;
//   msg.Gear_Enable_control = en;
//   msg.Gear_Shift_Req = valude;
//   msg.IPC_Mode_Shift = 0;
//   msg.IPC_Stop_Eme = 0;
//   Pack_Gear_Shift_Cmd_CAN_yeyazhidong_20210(&msg, data, &len, &ide);

//   autoware_can_msgs::CANPacket canpack;
//   canpack.id = (int)VehicleDataType::Gear_Cmd;
//   canpack.len = sizeof(data);
//   memcpy(&canpack.dat, data, sizeof(data));
//   can_box_pub_.publish(canpack);
}

// 设置手刹
void YunleVehicle::setFPB(uint8_t value, bool en)
{

//   uint8_t len;
//   uint8_t ide;
//   uint8_t data[8];
//   Parking_control_t msg;
//   msg.Parking_Enable_control = en;
//   msg.Parking_Cmd = value;

//   Pack_Parking_control_CAN_yeyazhidong_20210(&msg, data, &len, &ide);

//   autoware_can_msgs::CANPacket canpack;
//   canpack.id = (int)VehicleDataType::Parking_Cmd;
//   canpack.len = sizeof(data);
//   memcpy(&canpack.dat, data, sizeof(data));
//   can_box_pub_.publish(canpack);
}

void YunleVehicle::setSteerPostionWithLimit(int32_t value, int32_t limitAngle, int32_t limitSpdAngle)
{
}

// 设置杂项
void YunleVehicle::setMist()
{
}

// 设置转向灯
void YunleVehicle::setTurnLight(uint8_t valude)
{
}
// 设置大灯
void YunleVehicle::setBeamLight(uint8_t valude)
{
}
// 设置喇叭
void YunleVehicle::setBeep(uint8_t valude)
{
}
////////////////////// GET ////////////////////////////
void YunleVehicle::run()
{
//   can_info_pub_.publish(can_info_);
}

void YunleVehicle::getVcu_fbk(uint8_t *value)
{
//   uint8_t dlc_;
//   VCU_FeedBack_t _m;

//   Unpack_VCU_FeedBack_CAN_yeyazhidong_20210(&_m, value, dlc_);
//   // ROS_ERROR(" -----------------  %d,%d,%d,%d,%d\n",_m.Vehicle_Spd,_m.Steering_Ang,_m.Gear_Pos,_m.Brake_Tq,_m.Brake_pedal_Position);
//   getSpeed(_m.Vehicle_Spd);
//   getSteer(_m.Steering_Ang);
//   getGear(_m.Gear_Pos);
//   getAcc(_m.Brake_Tq);
//   getBrake(_m.Brake_pedal_Position);
}
void YunleVehicle::getBms_fbk(uint8_t *value)
{
    uint8_t dlc_;
    BMS_SOC_t _m;
    Unpack_BMS_SOC_BMS_for_C(&_m, value, dlc_);

    sensor_msgs::BatteryState bat_sta;
    bat_sta.header.stamp = ros::Time::now();
    bat_sta.voltage = _m.BMS_Total_VolBat/10.0;
    bat_sta.current = _m.BMS_current_Vol/10.0;
    bat_sta.percentage = _m.BMS_SOC/10.0;
    // ROS_ERROR("raw     %d  %d  %d  %d",_m.BMS_Total_VolBat,_m.BMS_current_Vol,_m.BMS_current_Cur,_m.BMS_SOC);
    batter_state_pub_.publish(bat_sta);
}
void YunleVehicle::getEnable_fbk(uint8_t *value)
{
//   uint8_t dlc_;
//   Enable_Fbk_t _m;
//   Unpack_Enable_Fbk_CAN_yeyazhidong_20210(&_m, value, dlc_);
//   getMode(_m.RC_Takerover_Flg);
}
void YunleVehicle::getErr_fbk(uint8_t *value)
{
//   uint8_t dlc_;
//   Err_Fbk_t _m;

//   Unpack_Err_Fbk_CAN_yeyazhidong_20210(&_m, value, dlc_);
}

void YunleVehicle::getMist(uint8_t *value)
{
}

bool YunleVehicle::check_ok(uint8_t *data)
{
}
void YunleVehicle::getMode(uint8_t value)
{
  can_info_.devmode = value;
}
void YunleVehicle::getAcc(uint16_t value)
{
//   // can_info_.devmode = 1;
//   can_info_.drivepedal = value * 0.19607;      /* 0nm ~ 50nm */
}
void YunleVehicle::getBrake(uint16_t value)
{
//   can_info_.brakepedal = value * 0.39215;      /* 0% ~ 100%*/
}
void YunleVehicle::getSteer(uint16_t value)
{
//   int16_t value_t = value;            // 无法号转有符号
//   value_t  = value_t < -32760?32760:value_t;                 // 底盘反馈有问题,做过滤
//   double value_ = ((value_t * 0.0008545 ) / 28.0 )* 540;  /* 0度 ~ 540度 */
//   can_info_.angle = value_;
//   // ROS_ERROR(" ===================== %ld,%d %f\n",value,value_t,value_);
//   // std::cout << value << std::endl;
}
void YunleVehicle::getGear(uint8_t value)
{
//   switch (value)
//   {
//   case GearShift::Parking:
//     can_info_.driveshift = autoware_msgs::Gear::PARK;
//     break;
//   case GearShift::Neutral:
//     can_info_.driveshift = autoware_msgs::Gear::NEUTRAL;
//     break;
//   case GearShift::Drive:
//     can_info_.driveshift = autoware_msgs::Gear::DRIVE;
//     break;
//   case GearShift::Reverse:
//     can_info_.driveshift = autoware_msgs::Gear::REVERSE;
//     break;
//   default:
//     can_info_.driveshift = autoware_msgs::Gear::NEUTRAL;
//     break;
//   }
}


void YunleVehicle::callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg)
{
  //can_info_.speed = msg->twist.linear.x * 3.6;   // km/h
  
}
void YunleVehicle::getSpeed(uint16_t value)
{
//    can_info_.speed = value * 0.01;   // km/h

//    //发布车辆反馈的速度
//    geometry_msgs::TwistStamped msg;
//    msg.header.stamp = ros::Time::now();
//    msg.twist.linear.x = can_info_.speed /3.6; // m/s
//    if(gear_mode == GearShift::Reverse)
//         msg.twist.linear.x *= -1;              // 后退是速度为负值

   
//    msg.twist.angular.z = can_info_.angle *(28.0/540.0)*(3.14159/180.);       // 角度值: 弧度
//    real_velocity_pub_.publish(msg);

//    if (backward_on)
//    { // 倒退计算距离
//        double time_sec = ros::Time::now().toNSec();
//        static double old_time = 0;
//        if (old_time != 0.0)
//        {
//            if (can_info_.speed < 0.1)
//                can_info_.speed = 0.0;
//            back_distance += (can_info_.speed / 3.6 * ((time_sec - old_time) / 1000000000.0));
//        }
//        old_time = time_sec;
//    }
}

void YunleVehicle::getTurnLight(uint8_t value)
{
//   can_info_.light = value;
}
void YunleVehicle::getBeamLight(uint8_t value)
{
  // can_info_.lamp = value;
}
void YunleVehicle::getBeep(uint8_t value)
{
}
