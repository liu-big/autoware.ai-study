#include "vehicle/simwuling.hpp"
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
SimWulingVehicle::SimWulingVehicle() : nh_(), private_nh_("~"),speed_ratio(1),set_mode(1)
{
  can_info_pub_ = nh_.advertise<autoware_can_msgs::CANInfo>("can_info", 1, true);
  can_box_pub_ = nh_.advertise<autoware_can_msgs::CANPacket>("can_box_cmd", 10, true);
   twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_cmd", 10, true);
  real_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("real_velocity", 10, true);
  current_velocity_sub_ = nh_.subscribe("/current_velocity", 10, &SimWulingVehicle::callbackCurrentVelocity, this);

  private_nh_.param<double>("/op_common_params/maxSteerAngle", maxsteerangle, 0.488);
  private_nh_.param<double>("add_speed_value", add_speed_value, 3);
  private_nh_.param<int>("rotate_speed", rotate_speed_, 800);
  private_nh_.param<double>("speed_gain", speed_gain, 1.6);
  private_nh_.param<double>("base_brake_value", base_brake_value, 35);


  spd_ratio_sub = nh_.subscribe("safe_range_ctrl", 10, &SimWulingVehicle::callbackSpeedRatio, this);

  _cnt = 0;
  dev_is_ok = false;
  fuzzy_PIDINT();
}

SimWulingVehicle::~SimWulingVehicle()
{
}


void SimWulingVehicle::callbackSpeedRatio(const autoware_msgs::Vehicle_Ctrl_StatusConstPtr& msg)
{
  speed_ratio = msg->safe_ctrl;
  m_aSafeDir[0] = msg->safe_from;
  m_aSafeDir[1] = msg->safe_lear;
}


void  SimWulingVehicle::fuzzy_PIDINT()
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

double SimWulingVehicle::pid_break(double obj_spd,double current_spd)
{
  speed_err=obj_spd-current_spd;

if(obj_spd != 0)
{
  if(speed_err<0)
  {
    brake_data+=0.1;
  }
  else
    brake_data = 0;

}
else
      brake_data+=0.2;
      // ROS_INFO("********************%f      %f    \n ",speed_err,brake_data);
    return 0;
}

float SimWulingVehicle::Speed_FilterOut(float target)
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
  // ROS_ERROR("---------------------%f",target_out);
	return  target_out;
}

void SimWulingVehicle::vehicle_cmd(autoware_msgs::VehicleCmd msg)
{

static int first_cont=0;
  if(dev_is_ok == false)
  {
    return ;
  }
  autoware_msgs::VehicleCmd _m;
 
  if(first_cont < 20){
	_m.mode = 0;
	first_cont++;
  }else{
  	_m.mode = set_mode;
  }

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

  int32_t _steer = abs(ang_to_steer) > abs(ang_to_steer1)? ang_to_steer:ang_to_steer1;
  _steer = (int(_steer / 10 )) * 10;
  int32_t _accel = abs(msg.twist_cmd.twist.linear.x) > abs(msg.accel_cmd.accel)?msg.twist_cmd.twist.linear.x:msg.accel_cmd.accel;
  int32_t _brake = msg.brake_cmd.brake/100;

  // 取平均值
  angle_.push(_steer);
  if(angle_.size() > 10){
	std::queue <int> tmp;
	tmp = angle_;
	_steer = 0;
	while(!tmp.empty()){
		_steer += tmp.front()/angle_.size();
		tmp.pop();
	}
	angle_.pop();
  }

  _m.accel_cmd.accel =  _accel;   // 0 ~ 100
  if(_steer>-540&&_steer<540)
    _m.steer_cmd.steer = - _steer ;    // -540 ~ 540
  _m.brake_cmd.brake = _brake;  // 0 ~ 100
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

  if(can_info_.driveshift == autoware_msgs::Gear::DRIVE){
    
     can_info_.speed = can_info_.speed<0?0:can_info_.speed;
    float  accel_ = (float)(speed_gain/(can_info_.speed / 3.6)  + add_speed_value);

      if(speed_ratio > 0 && speed_ratio <= 1){
        accel_ *=speed_ratio;
      }
      if((msg.ctrl_cmd.linear_velocity == 0.0 && msg.ctrl_cmd.linear_acceleration ==0.0 && msg.twist_cmd.twist.angular.z == 0.0) || speed_ratio == 0 || msg.twist_cmd.twist.linear.x <=0.2){
        accel_ = 0;
      _m.brake_cmd.brake = 35.0;
      }

      accel_ = accel_>35?35:accel_;
      _m.accel_cmd.accel = accel_;
  }else{
    _m.accel_cmd.accel  = 0;
     _m.brake_cmd.brake = 35.0; 
  }

// static float pid_data = 0,current_speed;
// if(can_info_.devmode)
// 	  pid_data	=fuzzypid.realize(80,can_info_.speed/10);//模糊PID处理数据
// //   ROS_ERROR("*********************%f   %f",pid_data,can_info_.speed);

//   if(can_info_.speed>800||pid_data<0){
//       pid_data = 0;
//   }
//   _m.accel_cmd.accel  =pid_data;

  if(can_info_.speed>rotate_speed_){
      _m.accel_cmd.accel  = 0;
  }

  ROS_ERROR("++++++++++++++++%d  %f",rotate_speed_,can_info_.speed);
// _m.accel_cmd.accel = pid_data ;
Speed_FilterOut(_m.accel_cmd.accel);
  setAccPercent(_m.accel_cmd.accel, true);
  setBrakePercent(_m.brake_cmd.brake,true);
  setSteerPostion(_m.steer_cmd.steer, true);
  if(_m.steer_cmd.steer > 100){
    _m.lamp_cmd.l  = 1;
    _m.lamp_cmd.r = 0;
  }else if(_m.steer_cmd.steer < -100){
    _m.lamp_cmd.l = 0;
    _m.lamp_cmd.r = 1 ;
  }else{
    _m.lamp_cmd.l = 0;
    _m.lamp_cmd.r = 0;
  }
  
  setGear(_m.gear_cmd.gear, true);
  //setTurnLight(_m.lamp_cmd.l + _m.lamp_cmd.r);
  setFPB(_m.emergency,false);
  increaseCNT();
  old_m = _m;
  ROS_INFO("------------ [can send] mode: %d, acc: %d, brake: %d, angle: %d, gear: %d,turnLight:%d",_m.mode,_m.accel_cmd.accel,_m.brake_cmd.brake,_m.steer_cmd.steer,_m.gear_cmd.gear,_m.lamp_cmd.l + _m.lamp_cmd.r);
  ROS_WARN ("------------ [can raw] mode: %d, acc: %d, brake: %d, angle: %f, gear: %d, speed:%f,turnLight:%d",can_info_.devmode,can_info_.drivepedal,can_info_.brakepedal,can_info_.angle,can_info_.driveshift,can_info_.speed,can_info_.light );
}
void SimWulingVehicle::vehicle_static(autoware_can_msgs::CANPacket msg)
{
  uint8_t data[8];//= msg.dat;
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
	
  // 通过真实车反馈的数据对车进行控制
  if(can_info_.devmode == 0){
	  geometry_msgs::TwistStamped ts;
	  ts.header.stamp = ros::Time::now();
	  ts.twist.linear.x = can_info_.drivepedal/10.0;
	
	  ts.twist.angular.z = -((can_info_.angle/28.0) * 3.14159 / 180.0);
	  twist_pub_.publish(ts);
  }
}

////////////////////// SET ////////////////////////////
  void SimWulingVehicle::increaseCNT() //  计数函数
  {
      _cnt++;
      if (_cnt > 15)
      {
          _cnt = 0;
      }
  }
  uint8_t SimWulingVehicle::calculateCtrlMSGCheckSum(void* canFrame) //校验合函数
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
bool SimWulingVehicle::setMode(int32_t en)
{
  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  MSG100_t msg;
  msg.AutoCtrlEna = en;
  msg.ModeCtrlCnt = _cnt;
  msg.ModeCtrlCks = 0; // calculateCtrlMSGCheckSum(&msg);
  Pack_MSG100_UserCAN2(&msg, data, &len, &ide);
  data[7] = calculateCtrlMSGCheckSum(data);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Mode_Cmd;
  canpack.len = len;
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

// 设置驱动百分比
void SimWulingVehicle::setAccPercent(int32_t value, bool en)
{
    uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  MSG101_t msg;
  msg.AccPedCmd =  (uint16_t)value*10;
  msg.AccCtrlEna = en;
  msg.AccPedInv =  (uint16_t)(1000 - value*10);
  msg.AccTkoDis = 0;
  msg.AccCtrlCnt = _cnt;
  msg.AccCtrlCks = 0; // calculateCtrlMSGCheckSum(&msg);
  Pack_MSG101_UserCAN2(&msg, data, &len, &ide);
  data[7] = calculateCtrlMSGCheckSum(data);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Acc_Cmd;
  canpack.len = len;
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}
void SimWulingVehicle::setBrakePercent(int32_t value, bool en)
{
  if (value > 1000)
      value = 1000;
  else if (value < 0)
      value = 0;

  uint8_t len;
  uint8_t ide;
  uint8_t data[8];
  MSG102_t msg;
  msg.BrkPedCmd = (uint16_t)value *10;
  msg.BrkCtrlEna = en;
  msg.BrkPedInv = (uint16_t)(1000 - value*10);
  msg.BrkTkoDis = 0;
  msg.BrkCtrlCnt = _cnt;
  msg.BrkCtrlCks = 0; // calculateCtrlMSGCheckSum(&msg);

  Pack_MSG102_UserCAN2(&msg, data, &len, &ide);
  data[7] = calculateCtrlMSGCheckSum(data);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Brake_Cmd;
  canpack.len = len;
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

// 设置制动百分比
void SimWulingVehicle::setSteerPostion(int32_t value, bool en)
{

  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  MSG103_t msg;
  msg.StrAngCmd = value * 10;
  msg.StrCtrlEna = en;
  msg.StrAngLimit = 520*10;     
  msg.StrAngRateLimit = 20*10; 
  msg.StrCtrlCnt = _cnt;
  msg.StrCtrlCks = 0; // calculateCtrlMSGCheckSum(&msg);
  Pack_MSG103_UserCAN2(&msg, data, &len, &ide);
  data[7] = calculateCtrlMSGCheckSum(data);

  // 封装can数据发布
  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Steer_Cmd;
  canpack.len = len;
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

// 设置档位
void SimWulingVehicle::setGear(uint8_t valude, bool en)
{
  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  MSG104_t msg;
  msg.GearCmd = valude;
  msg.GearCtrlEna = en;
  msg.TurnLightCmd = turn_val;
  msg.HornCmd = horn_val;
  msg.BeamLightCmd = beam_val;
  msg.GearCtrlCnt = _cnt;
  msg.GearCtrlCks = 0; // calculateCtrlMSGCheckSum(&msg);
  Pack_MSG104_UserCAN2(&msg, data, &len, &ide);
  data[7] = calculateCtrlMSGCheckSum(data);

  autoware_can_msgs::CANPacket canpack;
  canpack.id = (int)VehicleDataType::Gear_Cmd;
  canpack.len = len;
  memcpy(&canpack.dat, data, sizeof(data));
  can_box_pub_.publish(canpack);
}

// 设置手刹
void SimWulingVehicle::setFPB(uint8_t value, bool en)
{
}

// 设置转向灯
void SimWulingVehicle::setTurnLight(uint8_t valude)
{
    turn_val = valude;
}
// 设置大灯
void SimWulingVehicle::setBeamLight(uint8_t valude)
{
    beam_val = valude;
}
// 设置喇叭
void SimWulingVehicle::setBeep(uint8_t valude)
{
    horn_val = valude;
}
////////////////////// GET ////////////////////////////
bool SimWulingVehicle::check_ok(uint8_t *data)
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

void SimWulingVehicle::run()
{
  can_info_pub_.publish(can_info_);
}


void SimWulingVehicle::getMode(uint8_t *data)
{
  uint16_t value;
  MSG200_t msg;
  Unpack_MSG200_UserCAN2(&msg,data,0);
  value = msg.AutoCtrlStat;
  can_info_.devmode = value;
}
void SimWulingVehicle::getAcc(uint8_t *data)
{
  uint16_t value;
  MSG201_t msg;
  Unpack_MSG201_UserCAN2(&msg, data, 0);
  value = msg.AccPedExe;
  can_info_.drivepedal = value / 10.0 ;
  // can_info_.drivepedal *= geer_dir;
}
void SimWulingVehicle::getBrake(uint8_t *data)
{
  uint16_t value;
  MSG202_t msg;
  Unpack_MSG202_UserCAN2(&msg, data, 0);
  value = msg.BrkPedAct;
  can_info_.brakepedal = value/ 10.0;
}
void SimWulingVehicle::getSteer(uint8_t *data)
{
  int16_t value;
  MSG203_t msg;
  Unpack_MSG203_UserCAN2(&msg, data, 0);
  value = msg.StrAngAct;
  can_info_.angle = value/ 10.0;
}
void SimWulingVehicle::getGear(uint8_t *data)
{
  uint16_t value;
  MSG204_t msg;
  Unpack_MSG204_UserCAN2(&msg, data, 0);
  value = msg.GearAct;
  getTurnLight(msg.TurnLightAct);
  getBeamLight(msg.BeamLightAct);
  getBeep(msg.HornStatus);

  switch (value)
  {
  case GearShift::Parking:
    can_info_.driveshift = autoware_msgs::Gear::PARK;
    break;
  case GearShift::Neutral:
    can_info_.driveshift = autoware_msgs::Gear::NEUTRAL;
    break;
  case GearShift::Drive:
    geer_dir = 1;
    can_info_.driveshift = autoware_msgs::Gear::DRIVE;
    break;
  case GearShift::Reverse:
    geer_dir = -1;
    can_info_.driveshift = autoware_msgs::Gear::REVERSE;
    break;
  default:
    can_info_.driveshift = autoware_msgs::Gear::NEUTRAL;
    break;
  }
}


void SimWulingVehicle::callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg)
{
  // can_info_.speed = msg->twist.linear.x;
  
}
void SimWulingVehicle::getSpeed(uint8_t *data)
{
  can_info_.speed  = ((data[2] & (0xFFU)) << 8) | (data[1] & (0xFFU));
  // ROS_ERROR("hhhhhhhhhhhhhhhhh    %f    %d     %d",can_info_.speed,data[2],data[1]);
}

void SimWulingVehicle::getTurnLight(uint8_t value)
{
  static uint8_t old_light=can_info_.light ;
  can_info_.light = value;
    
  if ((value == 0) && (old_light !=can_info_.light)){
    pthread_t th;
    if (pthread_create(&th, nullptr, &SimWulingVehicle::set_auto_mode, this) != 0)
    {
      std::perror("pthread_create");
      std::exit(1);
    }
  }else if(value == 1){
    set_mode = 0;
  }
  old_light = can_info_.light;
}
void SimWulingVehicle::getBeamLight(uint8_t value)
{
  // can_info_.lamp = value;
}
void SimWulingVehicle::getBeep(uint8_t value)
{
  // can_info_.ev_mode = value;
}

// 按下喇叭启动自动模式
void *SimWulingVehicle::set_auto_mode(void *arg)
{
  ROS_ERROR("------ entry auto car mode ok");
  SimWulingVehicle *_this = (SimWulingVehicle *)arg;
  if( _this->can_info_.devmode == 0)
  {
    //_this->set_mode = 0;
    //usleep(1000000);
    _this->set_mode = 1;
  }
}