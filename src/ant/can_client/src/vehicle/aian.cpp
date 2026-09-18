#include "vehicle/aian.hpp"
#include"vehicle/fuzzy_PID.h"

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

static float brake_data=0,speed_err = 0;
FuzzyPID fuzzypid(1500,650,0.3,0.9,0.6,0.219,16.97,0.0);    //模糊PID初始化
AianVehicle::AianVehicle() : nh_(), private_nh_("~")
{
  can_info_pub_ = nh_.advertise<autoware_can_msgs::CANInfo>("can_info", 1, true);
  can_box_pub_ = nh_.advertise<autoware_can_msgs::CANPacket>("can_box_cmd", 10, true);
  
  real_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("real_velocity", 10, true);
  current_velocity_sub_ = nh_.subscribe("/current_velocity", 10, &AianVehicle::callbackCurrentVelocity, this);

  private_nh_.param<double>("/op_common_params/maxSteerAngle", maxsteerangle, 0.488);
  private_nh_.param<double>("add_speed_value", add_speed_value, 3);
  private_nh_.param<double>("speed_gain", speed_gain, 1.6);
  private_nh_.param<double>("base_brake_value", base_brake_value, 35);
  private_nh_.param<double>("stop_brake_increase", stop_brake_increase, 0.8);
  private_nh_.param<double>("normal_brake_increase", normal_brake_increase, 0.1);

  spd_ratio_sub = nh_.subscribe("safe_range_ctrl", 10, &AianVehicle::callbackSpeedRatio, this);

  _cnt = 0;
  dev_is_ok = false;
  fuzzy_PIDINT();
}

AianVehicle::~AianVehicle()
{
  
}


void AianVehicle::callbackSpeedRatio(const autoware_msgs::Vehicle_Ctrl_StatusConstPtr& msg)
{
  speed_ratio = msg->safe_ctrl;
  m_aSafeDir[0] = msg->safe_from;
  m_aSafeDir[1] = msg->safe_lear;
}


void  AianVehicle::fuzzy_PIDINT()
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


//刹车控制
double AianVehicle::pid_break(double obj_spd,double current_spd)
{
  speed_err=obj_spd-current_spd;

  if(obj_spd != 0)
  {
    if(speed_err<0)
    {
      brake_data+=normal_brake_increase;
      // if(speed_err<-4)
      //   brake_data+=normal_brake_increase*(current_spd/obj_spd);
    }
    else
      brake_data = 0;

  }
  else
        brake_data+=0.2;
        // ROS_INFO("********************%f      %f    \n ",speed_err,brake_data);
      return 0;
}

//速度平滑函数
float AianVehicle::Speed_FilterOut(float target)
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
  // ROS_ERROR("target_out  ---------------------%f",target_out);
	return  target_out;
}

void AianVehicle::vehicle_cmd(autoware_msgs::VehicleCmd msg)
{

  if(dev_is_ok == false)
  {
    return ;
  }
  autoware_msgs::VehicleCmd _m;
 
  _m.mode = set_mode;
  

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
  _steer = (int(_steer / 10 )) * 10;
  int32_t _accel = abs(msg.twist_cmd.twist.linear.x) > abs(msg.accel_cmd.accel)?msg.twist_cmd.twist.linear.x:msg.accel_cmd.accel;
  int32_t _brake = msg.brake_cmd.brake/100;

  _m.accel_cmd.accel =  _accel;   // 0 ~ 100
  _m.steer_cmd.steer = - _steer ;    // -540 ~ 540
  // _m.brake_cmd.brake = _brake;  // 0 ~ 100
  _m.lamp_cmd.l = 0;      // 0,1
  _m.lamp_cmd.r = 0;     // 0,1
  _m.gear_cmd.gear = autoware_msgs::Gear::DRIVE;   // 2,3,4
  _m.emergency = 0;


  if( can_info_.devmode == 1){
    _m.gear_cmd.gear = autoware_msgs::Gear::DRIVE;
  }else{
    _m.gear_cmd.gear = autoware_msgs::Gear::NEUTRAL;
  }
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




static float pid_data,current_speed,stop_brake_=0;
 float target;
if(can_info_.speed<0)
  can_info_.speed =0;
  current_speed = can_info_.speed;
  double _obj_speed = msg.twist_cmd.twist.linear.x*speed_gain;
  if(can_info_.devmode){
    if(speed_ratio == 0 ){  //是否有障碍物在安全距离内
        _obj_speed =  0.;
        stop_brake_ += stop_brake_increase;
        // ROS_ERROR("紧急制动!!!");
    }else{
      stop_brake_ = 0;
    }

    target = Speed_FilterOut(_obj_speed);//目标速度平滑处理
    if(current_speed>110)//五菱后溜底盘反馈从115开始往下减,此处防止后溜
    {
        current_speed = 0;
        target = _obj_speed;
    }


    if(target == 0){
        pid_data = 0;
        fuzzypid.init();
    }else{
        pid_data	=fuzzypid.realize(target,current_speed);//模糊PID处理数据
    }
      pid_break(_obj_speed,current_speed);//刹车控制

      _m.accel_cmd.accel = pid_data >60?60:pid_data;
      // _m.accel_cmd.accel = pid_data;
      if(_m.accel_cmd.accel<0)
        _m.accel_cmd.accel  = 0;
  }
  else{
        _m.accel_cmd.accel = 0;
        Speed_FilterOut(0);//确保退出自动驾驶时速度从零开始计算
        // fuzzypid.showInfo();
  }

  brake_data = std::max(stop_brake_,brake_data);
  _m.brake_cmd.brake = (int)brake_data;

  setAccPercent(_m.accel_cmd.accel, true);
  setBrakePercent(_m.brake_cmd.brake,true);
  setSteerPostion(_m.steer_cmd.steer, true);
  // #if 0
  if(_m.steer_cmd.steer > 100){
    _m.lamp_cmd.l  = 1;
    _m.lamp_cmd.r = 1;
  }else if(_m.steer_cmd.steer < -100){
    _m.lamp_cmd.l = 1;
    _m.lamp_cmd.r = 0 ;
  }else{
    _m.lamp_cmd.l = 0;
    _m.lamp_cmd.r = 0;
  }
  // #endif
  setGear(_m.gear_cmd.gear, true);
  setTurnLight(_m.lamp_cmd.l + _m.lamp_cmd.r);
  setFPB(_m.emergency,false);
  increaseCNT();
  old_m = _m;
  ROS_INFO("------------ [can send] mode: %d, acc: %d, brake: %d, angle: %d, gear: %d,speed:%f,turnLight:%d", _m.mode, _m.accel_cmd.accel, _m.brake_cmd.brake, _m.steer_cmd.steer, _m.gear_cmd.gear,target, _m.lamp_cmd.l + _m.lamp_cmd.r);
  ROS_WARN ("---------- [can raw] mode: %d, acc: %d, brake: %d, angle: %f, gear: %d, speed:%f,turnLight:%d",can_info_.devmode,can_info_.drivepedal,can_info_.brakepedal,can_info_.angle,can_info_.driveshift,can_info_.speed,can_info_.light );
}

void AianVehicle::vehicle_static(autoware_can_msgs::CANPacket msg)
{
  uint8_t data[8]; //= msg.dat;
  memcpy(data, &msg.dat, sizeof(msg.dat));
  switch (msg.id)
  {
  case VehicleDataType::Mode_fbk:
    dev_is_ok = true;
    getMode(data);
    break;
  case VehicleDataType::Acc_fbk:
    dev_is_ok = true;
    getAcc(data);
    break;
  case VehicleDataType::Brake_fbk:
    dev_is_ok = true;
    getBrake(data);
    break;
  case VehicleDataType::Steer_fbk:
    dev_is_ok = true;
    getSteer(data);
    break;
  case VehicleDataType::Gear_fbk:
    dev_is_ok = true;
    getGear(data);
    break;
  case VehicleDataType::Speed_fbk:
    dev_is_ok = true;
    getSpeed(data);
    break;

  default:
    break;
  }
}

////////////////////// SET ////////////////////////////
  void AianVehicle::increaseCNT() //  计数函数
  {
      _cnt++;
      if (_cnt > 15)
      {
          _cnt = 0;
      }
  }
  uint8_t AianVehicle::calculateCtrlMSGCheckSum(void* canFrame) //校验合函数
  {
      uint8_t sum = 0;
      uint8_t *ptr = (uint8_t *)canFrame;
      for (int i = 0; i < 8; i++)
      {
          sum += ptr[i];
      }
      return sum;
  }

// 设置驾驶模式
bool AianVehicle::setMode(int32_t en)
{
  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  IDS_DCU_MSG100_t msg;
  msg.IDS_DCU_AutoCtrlEna = en;
  msg.IDS_DCU_ModeCtrl_CTROLL = _cnt;
  msg.IDS_DCU_ModeCtrl_CHECKSUM = 0; // calculateCtrlMSGCheckSum(&msg);
  Pack_IDS_DCU_MSG100_testdb(&msg, data, &len, &ide);
  data[7] = calculateCtrlMSGCheckSum(data);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Mode_Cmd;
  canpack.len = sizeof(data);
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

// 设置驱动百分比
void AianVehicle::setAccPercent(int32_t value, bool en)
{
  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  IDS_DCU_MSG101_t msg;
  msg.IDS_DCU_AccPedCmd_ro =  (uint16_t)value*10;
  msg.IDS_DCU_AccCtrlEna = en;
  msg.IDS_DCU_AccPedInv_ro =  (uint16_t)(1000 - value*10);
  msg.IDS_DCU_AccOverRideDisable = 0;
  // msg.AccCtrlCnt = _cnt;
  // msg.AccCtrlCks = 0; // calculateCtrlMSGCheckSum(&msg);
  Pack_IDS_DCU_MSG101_testdb(&msg, data, &len, &ide);
  data[7] = calculateCtrlMSGCheckSum(data);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Acc_Cmd;
  canpack.len = sizeof(data);
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}
void AianVehicle::setBrakePercent(int32_t value, bool en)
{
  if (value > 1000)
      value = 1000;
  else if (value < 0)
      value = 0;

  uint8_t len;
  uint8_t ide;
  uint8_t data[8];
  IDS_DCU_MSG102_t msg;
  msg.IDS_DCU_BrkPedCmd_ro = (uint16_t)value *10;
  msg.IDS_DCU_BrkCtrlEna = en;
  msg.IDS_DCU_BrkPedInv_ro = (uint16_t)(1000 - value*10);
  msg.IDS_DCU_BrkOverRideDisable = 0;
  msg.IDS_DCU_BrkCtrl_CTROLL = _cnt;
  msg.IDS_DCU_BrkCtrl_CHECKSUM = 0; // calculateCtrlMSGCheckSum(&msg);

  Pack_IDS_DCU_MSG102_testdb(&msg, data, &len, &ide);
  data[7] = calculateCtrlMSGCheckSum(data);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Brake_Cmd;
  canpack.len = sizeof(data);
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

// 设置制动百分比
void AianVehicle::setSteerPostion(int32_t value, bool en)
{
  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  IDS_DCU_MSG103_t msg;
  msg.IDS_DCU_StrAngCmd_ro = value * 10;
  msg.IDS_DCU_StrCtrlEna = en;
  msg.IDS_DCU_StrAngLimit_ro = 520*10;     
  msg.IDS_DCU_StrAngRateLimit = 20*10; 
  msg.IDS_DCU_StrCtrl_CTROLL = _cnt;
  // msg.StrCtrlCks = 0; // calculateCtrlMSGCheckSum(&msg);
  Pack_IDS_DCU_MSG103_testdb(&msg, data, &len, &ide);
  data[7] = calculateCtrlMSGCheckSum(data);

  // 封装can数据发布
  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Steer_Cmd;
  canpack.len = sizeof(data);
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

// 设置档位
void AianVehicle::setGear(uint8_t valude, bool en)
{
  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  IDS_DCU_MSG104_t msg;
  msg.IDS_DCU_GearCmd = valude;
  msg.IDS_DCU_GearCtrlEna = en;
  msg.IDS_DCU_TurnLightCmd = turn_val;
  msg.IDS_DCU_HornCmd = horn_val;
  msg.IDS_DCU_BeamCmd = beam_val;
  msg.IDS_DCU_Gear_CTROLL = _cnt;
  msg.IDS_DCU_Gear_CHECKSUM = 0; // calculateCtrlMSGCheckSum(&msg);
  Pack_IDS_DCU_MSG104_testdb(&msg, data, &len, &ide);
  data[7] = calculateCtrlMSGCheckSum(data);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Gear_Cmd;
  canpack.len = sizeof(data);
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

// 设置手刹
void AianVehicle::setFPB(uint8_t value, bool en)
{
}

// 设置转向灯
void AianVehicle::setTurnLight(uint8_t valude)
{
    turn_val = valude;
}
// 设置大灯
void AianVehicle::setBeamLight(uint8_t valude)
{
    beam_val = valude;
}
// 设置喇叭
void AianVehicle::setBeep(uint8_t valude)
{
    horn_val = valude;
}
////////////////////// GET ////////////////////////////
bool AianVehicle::check_ok(uint8_t *data)
{
    uint8_t ch = data[7];
    data[7] = 0;
    uint8_t ret = calculateCtrlMSGCheckSum(data);
    if (ret != ch)
    {
        return false;
    }
    return true;
}

void AianVehicle::run()
{
  can_info_pub_.publish(can_info_);
}


void AianVehicle::getMode(uint8_t *data)
{
  uint16_t value;
  DCU_IDS_MSG200_t msg;
  Unpack_DCU_IDS_MSG200_testdb(&msg,data,0);
  value = msg.DCU_IDS_AutoCtrlStat;
  can_info_.devmode = value;
}
void AianVehicle::getAcc(uint8_t *data)
{
  uint16_t value;
  DCU_IDS_MSG201_t msg;
  Unpack_DCU_IDS_MSG201_testdb(&msg, data, 0);
  value = msg.DCU_IDS_AccPedExe_ro;
  can_info_.drivepedal = value / 10.0 ;
}
void AianVehicle::getBrake(uint8_t *data)
{
  uint16_t value;
  DCU_IDS_MSG202_t msg;
  Unpack_DCU_IDS_MSG202_testdb(&msg, data, 0);
  value = msg.DCU_IDS_BrkPedAct_ro;
  can_info_.brakepedal = value/ 10.0;
}
void AianVehicle::getSteer(uint8_t *data)
{
  int16_t value;
  DCU_IDS_MSG203_t msg;
  Unpack_DCU_IDS_MSG203_testdb(&msg, data, 0);
  value = msg.DCU_IDS_StrAngAct_ro;
  can_info_.angle = value/ 10.0;
}
void AianVehicle::getGear(uint8_t *data)
{
  uint16_t value;
  DCU_IDS_MSG204_t msg;
  Unpack_DCU_IDS_MSG204_testdb(&msg, data, 0);
  value = msg.DCU_IDS_GearAct;
  getTurnLight(msg.DCU_IDS_TurnLightAct);
  // getBeamLight(msg.BeamLightAct);//远近灯光状态
  getBeep(msg.DCU_IDS_HornStatus);
  // getSpeed(msg.VehSpd);

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


void AianVehicle::callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg)
{
  // can_info_.speed = msg->twist.linear.x;
}

void AianVehicle::getSpeed(uint8_t *data)
{
  int16_t value;
  DCU_IDS_MSG206_t msg;
  Unpack_DCU_IDS_MSG206_testdb(&msg, data, 0);
  value = msg.DCU_IDS_VehSpd_ro;

  can_info_.speed = value * 0.01;   // km/h

  geometry_msgs::TwistStamped msg_pub;
  msg_pub.header.stamp = ros::Time::now();
  msg_pub.twist.linear.x = can_info_.speed; // m/s
  real_velocity_pub_.publish(msg_pub);
}

void AianVehicle::getTurnLight(uint8_t value)
{
  static uint8_t old_light=can_info_.light ;
  can_info_.light = value;
    
  if(can_info_.devmode!=1)
    if ((value == 0) && (old_light !=can_info_.light)){
      pthread_t th;
      if (pthread_create(&th, nullptr, &AianVehicle::set_auto_mode, this) != 0)
      {
        std::perror("pthread_create");
        std::exit(1);
      }
    }else if(value == 1){
      set_mode = 0;
    }
  old_light = can_info_.light;
}
void AianVehicle::getBeamLight(uint8_t value)
{
  // can_info_.lamp = value;
}
void AianVehicle::getBeep(uint8_t value)
{
  can_info_.ev_mode = value;
}

// 按下喇叭启动自动模式
void *AianVehicle::set_auto_mode(void *arg)
{
  ROS_ERROR("------ entry auto car mode ok");
  AianVehicle *_this = (AianVehicle *)arg;
  if( _this->can_info_.devmode == 0)
  {
    //_this->set_mode = 0;
    //usleep(1000000);
    _this->set_mode = 1;
  }
}