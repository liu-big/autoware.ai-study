#include "vehicle/zhongyun.hpp"
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

ZhonyunVehicle::ZhonyunVehicle() : nh_(), private_nh_("~")
{
  can_info_pub_ = nh_.advertise<autoware_can_msgs::CANInfo>("can_info", 1, true);
  can_box_pub_ = nh_.advertise<autoware_can_msgs::CANPacket>("can_box_cmd", 10, true);
  batter_state_pub_ = nh_.advertise<sensor_msgs::BatteryState>("battery_state", 10, true);
  real_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("real_velocity", 10, true);
  current_velocity_sub_ = nh_.subscribe("/current_velocity", 10, &ZhonyunVehicle::callbackCurrentVelocity, this);

  sub_final_waypoints_ = nh_.subscribe("/final_waypoints", 1, &ZhonyunVehicle::callbackFinalWaypoints, this);


  sub_Trajectory_Cost = nh_.subscribe("/local_trajectory_cost", 1, &ZhonyunVehicle::callbackGetLocalTrajectoryCost, this);

  private_nh_.param<double>("/op_common_params/maxSteerAngle", maxsteerangle, 0.488);
  private_nh_.param<double>("/op_common_params/maxDistanceToAvoid", maxdistancetoavoid, 5);
  private_nh_.param<double>("add_speed_value", add_speed_value, 3);
  private_nh_.param<double>("speed_gain", speed_gain, 1.6);
  private_nh_.param<double>("base_brake_value", base_brake_value, 35);

  private_nh_.param<double>("stop_brake_increase", stop_brake_increase, 0.1);
  private_nh_.param<double>("normal_brake_increase", normal_brake_increase, 0.8);
  private_nh_.param<bool>("enable_recover", enable_recover, false);

  spd_ratio_sub = nh_.subscribe("safe_range_ctrl", 1, &ZhonyunVehicle::callbackSpeedRatio, this);
  sub_GoalRemainingDistance = nh_.subscribe("/goal_remaining_distance", 1, &ZhonyunVehicle::callbackGetGoalRemainingDistance, this);
  speed_ratio = 1.0;
  fuzzy_PIDINT();

  ctrl_mode = CtrlMode::Normal;
}

ZhonyunVehicle::~ZhonyunVehicle()
{
}

void ZhonyunVehicle::callbackFinalWaypoints(const autoware_msgs::Lane &final_waypoints)
{
    m_nTargetNum = final_waypoints.waypoints.size();
}

void ZhonyunVehicle::callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr& msg)
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

void ZhonyunVehicle::callbackGetGoalRemainingDistance(const std_msgs::Float32ConstPtr &msg)
{
    m_fRemainingDistance = msg->data;
}

void ZhonyunVehicle::callbackSpeedRatio(const autoware_msgs::Vehicle_Ctrl_StatusConstPtr& msg)
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

void ZhonyunVehicle::fuzzy_PIDINT()
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

double ZhonyunVehicle::pid_break(double obj_spd, double current_spd)
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

float ZhonyunVehicle::Speed_FilterOut(float target)
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


void ZhonyunVehicle::vehicle_cmd(autoware_msgs::VehicleCmd msg)
{
  autoware_msgs::VehicleCmd _m;
  _m.mode = 1;

  double ang_t = msg.twist_cmd.twist.angular.z;
  double ang_t1= msg.ctrl_cmd.steering_angle;

    if(ang_t > maxsteerangle)
    ang_t = maxsteerangle;
  else if(ang_t < -maxsteerangle)
    ang_t = -maxsteerangle;

    if(ang_t1 > maxsteerangle)
    ang_t = maxsteerangle;
  else if(ang_t1 < -maxsteerangle)
    ang_t1 = -maxsteerangle;

  double ang_to_steer = ang_t *( 180 /3.1415) * ( 540 / (maxsteerangle * ( 180 /3.1415)) ) ;    // 弧度 转为度 * 每度对应方向盘的大小 = 当前方向盘角度
  double ang_to_steer1= ang_t1 *( 180 /3.1415) * ( 540 / (maxsteerangle * ( 180 /3.1415)) ) ;  

  int32_t _steer = ang_to_steer1;// abs(ang_to_steer) > abs(ang_to_steer1)? ang_to_steer:ang_to_steer1;
//   _steer = (int(_steer / 10 )) * 10;
  _steer = (int(_steer));
  int32_t _accel = abs(msg.twist_cmd.twist.linear.x) > abs(msg.accel_cmd.accel)?msg.twist_cmd.twist.linear.x:msg.accel_cmd.accel;
  int32_t _brake = msg.brake_cmd.brake/100;

  _m.accel_cmd.accel =  _accel;   // 0 ~ 100
  _m.steer_cmd.steer = _steer ;    // -540 ~ 540
  _m.brake_cmd.brake = _brake;  // 0 ~ 100
  _m.lamp_cmd.l = 0;      // 0,1
  _m.lamp_cmd.r = 0;     // 0,1
  _m.gear_cmd.gear = autoware_msgs::Gear::DRIVE;   // 2,3,4
  _m.emergency = 0;

  switch (_m.gear_cmd.gear)
  {
  case autoware_msgs::Gear::PARK:
    _m.gear_cmd.gear = GearShift::Parking;
    break;
  case autoware_msgs::Gear::NEUTRAL:
    _m.gear_cmd.gear = GearShift::Neutral;
    break;
  case autoware_msgs::Gear::DRIVE:
    _m.gear_cmd.gear = GearShift::Drive;
    break;
  case autoware_msgs::Gear::REVERSE:
    _m.gear_cmd.gear = GearShift::Reverse;
    break;
  default:
    _m.gear_cmd.gear = GearShift::Neutral;
    break;
  }

  setMode(_m.mode);

  static double dTempBreake = base_brake_value;
  if(1){

    can_info_.speed = can_info_.speed<0?0:can_info_.speed;

    float  accel_ = (float)(speed_gain/(can_info_.speed / 3.6)  + add_speed_value);

    if(speed_ratio > 0 && speed_ratio <= 1){
      _m.accel_cmd.accel *=speed_ratio;
    }      
    static int count_=0;
    if((msg.ctrl_cmd.linear_velocity == 0.0 && msg.ctrl_cmd.linear_acceleration ==0.0 && msg.twist_cmd.twist.angular.z == 0.0) || speed_ratio == 0 || msg.twist_cmd.twist.linear.x <=0.2){
        accel_ = 0;
        dTempBreake += 0.001;
      }else{
          dTempBreake = base_brake_value;
      }

      accel_ = accel_>30?30:accel_;
      _m.accel_cmd.accel = accel_;
  }else{
    _m.accel_cmd.accel  = 0;
    //  _m.brake_cmd.brake = base_brake_value; 
  }
  
  static float pid_data,current_speed;
  if(can_info_.speed<0)
    can_info_.speed =0;
  current_speed = can_info_.speed;
  double _obj_speed = msg.twist_cmd.twist.linear.x*speed_gain;
  if (gear_reverse_on)// && can_info_.driveshift != 4)
  {
      _m.gear_cmd.gear = GearShift::Reverse;
      _m.steer_cmd.steer = -_m.steer_cmd.steer;  // 1/2角度方向旋转
      _obj_speed = 1.5;
      //   _m.accel_cmd.accel = 4;
      //   _m.brake_cmd.brake = 0;
  }

  static float stop_brake_=0;
  if(can_info_.devmode && can_info_.driveshift != 1){
    // float pid_data = pid_ctrl(msg.twist_cmd.twist.linear.x,current_speed);//
    if(_obj_speed <3&& _obj_speed !=0)//限制转弯速度过小
      _obj_speed = 3;

    float target = Speed_FilterOut(_obj_speed);//目标速度平滑处理
    if(current_speed>110)//五菱后溜底盘反馈从115开始往下减,此处防止后溜
    {
     current_speed = 0;
     target = _obj_speed;
    }

    if(speed_ratio == 0){
        target =  0.;
        stop_brake_ += stop_brake_increase;
    }else{
      stop_brake_ = 0;
    }

    static int num=0;
    if(can_info_.driveshift == 2 && gear_reverse_on == false)
    {
      num++;
        if(num > 2)
        {
          current_speed = -current_speed;
         target = _obj_speed*10;
         brake_data = 80;
		 }
    }
    else{
        num = 0;
        pid_break(_obj_speed,current_speed);
    }
	
    if(target == 0){
      pid_data = 0;
      fuzzypid.init();
    //  fuzzypid.ResetPID(1500, 650, 0.3, 0.9, 0.6, 0.219, 16.97, 0.0); //模糊PID初始化
    }else{
      pid_data	=fuzzypid.realize(target,current_speed);//模糊PID处理数据
    }
    // pid_break(_obj_speed,current_speed);

    // ROS_WARN("Test_Data------    %f      ---%f    ---%f",_obj_speed,current_speed,pid_data);
    _m.accel_cmd.accel = pid_data >60?60:pid_data;
    // _m.accel_cmd.accel = pid_data;
    if(_m.accel_cmd.accel<0)
        _m.accel_cmd.accel  = 0;
  }else{
    _m.accel_cmd.accel = 0;
    pid_.ActualSpeed = 0;
    Speed_FilterOut(0);//确保退出自动驾驶时速度从零开始计算
    // fuzzypid.showInfo();
  }

   brake_data = std::max(stop_brake_,brake_data);
  if(brake_data < 0){
      brake_data = 0;
  }else if(brake_data > 100){
      brake_data = 100;
  }
  _m.brake_cmd.brake = (int)brake_data;


  setAccPercent(_m.accel_cmd.accel, true);
  if(_m.brake_cmd.brake > 0){
    setBrakePercent(_m.brake_cmd.brake, true);
  }else{
    setBrakePercent(_m.brake_cmd.brake, false);
  }

  if(_m.steer_cmd.steer > 250){
    _m.lamp_cmd.l  = 1;
    _m.lamp_cmd.r = 0;
      
  }else if(_m.steer_cmd.steer < -250){
    _m.lamp_cmd.l = 0;
    _m.lamp_cmd.r = 1 ;
  }else{
    _m.lamp_cmd.l = 0;
    _m.lamp_cmd.r = 0;
  }

  if(_m.steer_cmd.steer > 539){
      _m.steer_cmd.steer = 539;
  }else if(_m.steer_cmd.steer  < -539){
    _m.steer_cmd.steer = -539;
  }

  setSteerPostion(_m.steer_cmd.steer, true);

  
  gear_mode = _m.gear_cmd.gear;
  setGear(_m.gear_cmd.gear, true);
  setTurnLight(_m.lamp_cmd.l + _m.lamp_cmd.r);
  setFPB(_m.emergency,false);
  old_m = _m;  
  ROS_INFO("------------ [can send] mode: %d, acc: %d, brake: %d, angle: %d, gear: %d,turnLight:%d",_m.mode,_m.accel_cmd.accel,_m.brake_cmd.brake,_m.steer_cmd.steer,_m.gear_cmd.gear,_m.lamp_cmd.l + _m.lamp_cmd.r);
  ROS_WARN ("------------ [can raw] mode: %d, acc: %d, brake: %d, angle: %f, gear: %d, speed:%f,turnLight:%d",can_info_.devmode,can_info_.drivepedal,can_info_.brakepedal,can_info_.angle,can_info_.driveshift,can_info_.speed,can_info_.light );
}

void ZhonyunVehicle::vehicle_static(autoware_can_msgs::CANPacket msg)
{
  uint8_t data[8];//= msg.dat;
  memcpy(data, &msg.dat, sizeof(msg.dat)); 
  switch (msg.id)
  {
    case VehicleDataType::Vcu_fbk:
      getVcu_fbk(data);
      break;
    case VehicleDataType::Bms_fbk:
      getBms_fbk(data);
      break;
    case VehicleDataType::Enable_fbk:
      getEnable_fbk(data);
      break;
    case VehicleDataType::Err_fbk:
      getErr_fbk(data);
      break;

    default:
      break;
  }

}

////////////////////// SET ////////////////////////////
// 设置驾驶模式
bool ZhonyunVehicle::setMode(int32_t en)
{
  autoware_can_msgs::CANPacket msg;

}

// 设置驱动百分比
void ZhonyunVehicle::setAccPercent(int32_t value, bool en)
{
  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  Drive_control_t msg;
  msg.Driven_Enable_control = en;


  value = (value / 0.001525);
  if (value > 65535)
    value = 65535;
  else if (value < 0)
    value = 0;
  msg.Drive_Tq_Req = (uint16_t)value;

  Pack_Drive_control_CAN_yeyazhidong_20210(&msg, data, &len, &ide);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Drive_Cmd;
  canpack.len = sizeof(data);
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}
void ZhonyunVehicle::setBrakePercent(int32_t value, bool en)
{

  uint8_t len;
  uint8_t ide;
  uint8_t data[8];
  brake_control_t msg;
  msg.Brake_Enable_control = en;
  value = (value / 0.001525);
  if (value > 65535)
    value = 65535;
  else if (value < 0)
    value = 0;

  msg.Brake_Tq_Req = (uint16_t)value;

  Pack_brake_control_CAN_yeyazhidong_20210(&msg, data, &len, &ide);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Brake_Cmd;
  canpack.len = sizeof(data);
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

// 设置制动百分比
void ZhonyunVehicle::setSteerPostion(int32_t value, bool en)
{

  value = (((value / 540.0 * (maxsteerangle * ( 180 /3.1415))) + (maxsteerangle * ( 180 /3.1415))) / 0.0008545);
  if (value > 65535)
    value = 65535;
  else if (value < 0)
    value = 0;

  uint8_t len;
  uint8_t ide;
  uint8_t data[8];
  steering_control_t msg;
  msg.Steering_Enable_control = en;
  //  通过can协议移位得到data[8]数据
  msg.Steering_Pos_Req = value;

  Pack_steering_control_CAN_yeyazhidong_20210(&msg, data, &len, &ide);

  // 封装can数据发布
  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Steer_Cmd;
  canpack.len = sizeof(data);
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

// 设置档位
void ZhonyunVehicle::setGear(uint8_t valude, bool en)
{
  uint8_t len;
  uint8_t ide;
  uint8_t data[8];
  Gear_Shift_Cmd_t msg;
  msg.Gear_Enable_control = en;
  msg.Gear_Shift_Req = valude;
  msg.IPC_Mode_Shift = 0;
  msg.IPC_Stop_Eme = 0;
  Pack_Gear_Shift_Cmd_CAN_yeyazhidong_20210(&msg, data, &len, &ide);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Gear_Cmd;
  canpack.len = sizeof(data);
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

// 设置手刹
void ZhonyunVehicle::setFPB(uint8_t value, bool en)
{

  uint8_t len;
  uint8_t ide;
  uint8_t data[8];
  Parking_control_t msg;
  msg.Parking_Enable_control = en;
  msg.Parking_Cmd = value;

  Pack_Parking_control_CAN_yeyazhidong_20210(&msg, data, &len, &ide);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Parking_Cmd;
  canpack.len = sizeof(data);
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

void ZhonyunVehicle::setSteerPostionWithLimit(int32_t value, int32_t limitAngle, int32_t limitSpdAngle)
{
}

// 设置杂项
void ZhonyunVehicle::setMist()
{
}

// 设置转向灯
void ZhonyunVehicle::setTurnLight(uint8_t valude)
{
}
// 设置大灯
void ZhonyunVehicle::setBeamLight(uint8_t valude)
{
}
// 设置喇叭
void ZhonyunVehicle::setBeep(uint8_t valude)
{
}
////////////////////// GET ////////////////////////////
void ZhonyunVehicle::run()
{
  can_info_pub_.publish(can_info_);
}

void ZhonyunVehicle::getVcu_fbk(uint8_t *value)
{
  uint8_t dlc_;
  VCU_FeedBack_t _m;

  Unpack_VCU_FeedBack_CAN_yeyazhidong_20210(&_m, value, dlc_);
  // ROS_ERROR(" -----------------  %d,%d,%d,%d,%d\n",_m.Vehicle_Spd,_m.Steering_Ang,_m.Gear_Pos,_m.Brake_Tq,_m.Brake_pedal_Position);
  getSpeed(_m.Vehicle_Spd);
  getSteer(_m.Steering_Ang);
  getGear(_m.Gear_Pos);
  getAcc(_m.Brake_Tq);
  getBrake(_m.Brake_pedal_Position);
}
void ZhonyunVehicle::getBms_fbk(uint8_t *value)
{
  uint8_t dlc_;
  BMS_FeedBack_t _m;
  Unpack_BMS_FeedBack_CAN_yeyazhidong_20210(&_m, value, dlc_);

  sensor_msgs::BatteryState bat_sta;
  bat_sta.header.stamp = ros::Time::now();
  bat_sta.voltage = _m.Vol*0.1;
  bat_sta.current = _m.Current;
  bat_sta.percentage = _m.Soc;
  batter_state_pub_.publish(bat_sta);
}
void ZhonyunVehicle::getEnable_fbk(uint8_t *value)
{
  uint8_t dlc_;
  Enable_Fbk_t _m;
  Unpack_Enable_Fbk_CAN_yeyazhidong_20210(&_m, value, dlc_);
  getMode(_m.RC_Takerover_Flg);
}
void ZhonyunVehicle::getErr_fbk(uint8_t *value)
{
  uint8_t dlc_;
  Err_Fbk_t _m;

  Unpack_Err_Fbk_CAN_yeyazhidong_20210(&_m, value, dlc_);
}

void ZhonyunVehicle::getMist(uint8_t *value)
{
}

bool ZhonyunVehicle::check_ok(uint8_t *data)
{
}
void ZhonyunVehicle::getMode(uint8_t value)
{
  can_info_.devmode = value;
}
void ZhonyunVehicle::getAcc(uint16_t value)
{
  // can_info_.devmode = 1;
  can_info_.drivepedal = value * 0.19607;      /* 0nm ~ 50nm */
}
void ZhonyunVehicle::getBrake(uint16_t value)
{
  can_info_.brakepedal = value * 0.39215;      /* 0% ~ 100%*/
}
void ZhonyunVehicle::getSteer(uint16_t value)
{
  int16_t value_t = value;            // 无法号转有符号
  value_t  = value_t < -32760?32760:value_t;                 // 底盘反馈有问题,做过滤
  double value_ = ((value_t * 0.0008545 ) / 28.0 )* 540;  /* 0度 ~ 540度 */
  can_info_.angle = value_;
  // ROS_ERROR(" ===================== %ld,%d %f\n",value,value_t,value_);
  // std::cout << value << std::endl;
}
void ZhonyunVehicle::getGear(uint8_t value)
{
  switch (value)
  {
  case GearShift::Parking:
    can_info_.driveshift = autoware_msgs::Gear::PARK;
    break;
  case GearShift::Neutral:
    can_info_.driveshift = autoware_msgs::Gear::NEUTRAL;
    break;
  case GearShift::Drive:
    can_info_.driveshift = autoware_msgs::Gear::DRIVE;
    break;
  case GearShift::Reverse:
    can_info_.driveshift = autoware_msgs::Gear::REVERSE;
    break;
  default:
    can_info_.driveshift = autoware_msgs::Gear::NEUTRAL;
    break;
  }
}


void ZhonyunVehicle::callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg)
{
  //can_info_.speed = msg->twist.linear.x * 3.6;   // km/h
  
}
void ZhonyunVehicle::getSpeed(uint16_t value)
{
   can_info_.speed = value * 0.01;   // km/h

   //发布车辆反馈的速度
   geometry_msgs::TwistStamped msg;
   msg.header.stamp = ros::Time::now();
   msg.twist.linear.x = can_info_.speed /3.6; // m/s
   if(gear_mode == GearShift::Reverse)
        msg.twist.linear.x *= -1;              // 后退是速度为负值

   
   msg.twist.angular.z = can_info_.angle *(28.0/540.0)*(3.14159/180.);       // 角度值: 弧度
   real_velocity_pub_.publish(msg);

   if (backward_on)
   { // 倒退计算距离
       double time_sec = ros::Time::now().toNSec();
       static double old_time = 0;
       if (old_time != 0.0)
       {
           if (can_info_.speed < 0.1)
               can_info_.speed = 0.0;
           back_distance += (can_info_.speed / 3.6 * ((time_sec - old_time) / 1000000000.0));
       }
       old_time = time_sec;
   }
}

void ZhonyunVehicle::getTurnLight(uint8_t value)
{
  can_info_.light = value;
}
void ZhonyunVehicle::getBeamLight(uint8_t value)
{
  // can_info_.lamp = value;
}
void ZhonyunVehicle::getBeep(uint8_t value)
{
}
