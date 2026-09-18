#include "vehicle/kaiwo.hpp"
#include"vehicle/fuzzy_PID.h"

KaiWo::KaiWo() : nh_(), private_nh_("~")
{
  can_info_pub_ = nh_.advertise<autoware_can_msgs::CANInfo>("can_info", 1, true);
  can_box_pub_ = nh_.advertise<autoware_can_msgs::CANPacket>("can_box_cmd", 10, true);
  goal_on_pub_ = nh_.advertise<std_msgs::Bool>("goal_on", 1, true);
  
  real_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("real_velocity", 10, true);
  current_velocity_sub_ = nh_.subscribe("/current_velocity", 10, &KaiWo::callbackCurrentVelocity, this);
  sub_current_behavior = nh_.subscribe("/current_behavior", 1, &KaiWo::callbackGetBehaviorState, this);
  brake_mode_sub_ = nh_.subscribe("/vehicle_brake_mode", 1, &KaiWo::brakeNodeCallback, this);


  sub_GoalRemainingDistance = nh_.subscribe("/goal_remaining_distance", 1, &KaiWo::callbackGetGoalRemainingDistance, this);

  private_nh_.param<double>("/op_common_params/rollOutsNumber", m_rollOuts_number, 1);
  private_nh_.param<double>("/op_common_params/maxSteerAngle", maxsteerangle, 0.488);
  private_nh_.param<double>("/op_common_params/maxDistanceToAvoid", maxdistancetoavoid, 7);
  private_nh_.param<double>("/op_common_params/minFollowingDistance", minFollowingDistance, 10);
  private_nh_.param<double>("/op_common_params/maxVelocity", maxvelocity, 5.0);
  private_nh_.param<double>("speed_gain", speed_gain, 1.6);

  _cnt = 0;
  dev_is_ok = false;
  
  speed_pid_controller = new control::PIDController(0.5, 0.02, 0.01,-100.0,80.0);
  b_local_status = true;
  m_brake_mode = 0;
  driver_state = INITIAL_STATE;
  door_val = 0;      // 对门无动作
  // clew_tone_type = 0; // 提示音无动作
  door_val_flag = false;
  clew_tone_flag = false;
}

KaiWo::~KaiWo()
{
}

void KaiWo::callbackGetGoalRemainingDistance(const std_msgs::Float32ConstPtr &msg)
{
    m_fRemainingDistance = msg->data;
    if(m_fRemainingDistance > 20){
      driver_state = FORWARD_STATE;

      door_val = 2;
    }else{
      driver_state = PULL_OVER;
      if (can_info_.speed == 0 && m_fRemainingDistance <= 8)
      {
        door_val = 1;
        driver_state = FINISH_STATE;
      }
    }

    static int pre_state = driver_state;
    if(pre_state != driver_state){
      clew_tone_flag = true;
      door_val_flag = true;
    }
    ROS_ERROR("============================ pre_state: %d, driver_state: %d, clew_tone_flag: %d, door_val_flag: %d\n", pre_state, driver_state,clew_tone_flag,door_val_flag);

    pre_state = driver_state;
}


void KaiWo::brakeNodeCallback(const std_msgs::Float32::ConstPtr & msg)
{
  m_brake_mode = msg->data;
}

void KaiWo::callbackGetBehaviorState(const geometry_msgs::TwistStampedConstPtr& msg)
{

  // if (msg->twist.angular.y == INITIAL_STATE)
  //   driver_state = INITIAL_STATE;
  // else if(msg->twist.angular.y == WAITING_STATE)
  //   driver_state = WAITING_STATE;
  // else if(msg->twist.angular.y == FORWARD_STATE)
  //   driver_state = FORWARD_STATE;
  // else if(msg->twist.angular.y == STOPPING_STATE)
  //   driver_state = STOPPING_STATE;
  // else if(msg->twist.angular.y == EMERGENCY_STATE)
  //   driver_state = EMERGENCY_STATE;
  // else if(msg->twist.angular.y == TRAFFIC_LIGHT_STOP_STATE)
  //   driver_state = TRAFFIC_LIGHT_STOP_STATE;
  // else if(msg->twist.angular.y == STOP_SIGN_STOP_STATE)
  //   driver_state = STOP_SIGN_STOP_STATE;
  // else if(msg->twist.angular.y == STOP_SIGN_WAIT_STATE)
  //   driver_state = STOP_SIGN_WAIT_STATE;
  // else if(msg->twist.angular.y == FOLLOW_STATE)
  //   driver_state = FOLLOW_STATE;
  // else if(msg->twist.angular.y == LANE_CHANGE_STATE)
  //   driver_state = LANE_CHANGE_STATE;
  // else if(msg->twist.angular.y == OBSTACLE_AVOIDANCE_STATE)
  //   driver_state = OBSTACLE_AVOIDANCE_STATE;
  // else if(msg->twist.angular.y == FINISH_STATE)
  //   driver_state = FINISH_STATE;


  // static int pre_state = driver_state;
  // if(pre_state != driver_state && driver_state == FINISH_STATE){
  //   door_val = 1;   // 开门
  //   clew_tone_type = 0xff;      // 关闭提示音g
  //   printf("111111111111111111111111\n");
  // //}else if(pre_state != driver_state && driver_state == INITIAL_STATE){
  // }else if(pre_state != driver_state && pre_state == FINISH_STATE){
  //   printf("22222222222222222222222222222\n");
  //   door_val = 2;   // 关门
  //   clew_tone_type = 3;        // 叮咚的聲音
  // }

  // pre_state = driver_state;
}

void KaiWo::vehicle_cmd(autoware_msgs::VehicleCmd msg)
{

  m_object_speed = msg.twist_cmd.twist.linear.x * speed_gain;

  if (dev_is_ok == false)
  {
    return ;
  }
  
  autoware_msgs::VehicleCmd _m;
 
  _m.mode = 1;//set_mode;
  
  double ang_tmp = msg.ctrl_cmd.steering_angle; // msg.twist_cmd.twist.angular.z;

  if(ang_tmp > maxsteerangle)
    ang_tmp = maxsteerangle;
  else if(ang_tmp < -maxsteerangle)
    ang_tmp = -maxsteerangle;

  double ang_to_steer = ang_tmp *( 180 /3.1415) * ( 540 / (maxsteerangle * ( 180 /3.1415)) ) ;    // 弧度 转为度 * 每度对应方向盘的大小 = 当前方向盘角度

  int32_t _brake = msg.brake_cmd.brake/100;

  //_m.accel_cmd.accel =  _accel;   // 0 ~ 100
  _m.steer_cmd.steer = - ang_to_steer ;    // -540 ~ 540
  // _m.brake_cmd.brake = _brake;  // 0 ~ 100
  _m.lamp_cmd.l = 0;      // 0,1
  _m.lamp_cmd.r = 0;      // 0,1
  _m.gear_cmd.gear = autoware_msgs::Gear::DRIVE;   // 2,3,4
  _m.emergency = 0;

  // if( can_info_.devmode == 1){
  //   _m.gear_cmd.gear = autoware_msgs::Gear::DRIVE;
  // }else{
  //   _m.gear_cmd.gear = autoware_msgs::Gear::NEUTRAL;
  // }

  switch (_m.gear_cmd.gear)
  {
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

  // setMode(_m.mode);
 
  if(can_info_.devmode == 1){
  double current_speed = can_info_.speed;
  double diff = m_object_speed - current_speed;

  double acceleration_cmd = speed_pid_controller->Control(diff, 0.1);


  static double brake_count = 0;
  if(m_brake_mode == 2){            
    brake_count += 0.2;
    acceleration_cmd = -brake_count - 3;
    speed_pid_controller->Reset();
  }else if(m_brake_mode == 1){
    if( current_speed > 2 )
    {
      brake_count += 0.05;
      acceleration_cmd = -brake_count - 1;
    }
  }else{
    if(msg.twist_cmd.twist.linear.x == 0){
      brake_count += 0.02;
      acceleration_cmd = -brake_count - 0;
    }else{
      brake_count = 0;
    }
  }

  if(acceleration_cmd > 100 ){
    acceleration_cmd = 100;
  }else if(acceleration_cmd < -50 ){
    acceleration_cmd = -50;
  }

  m_acceleration_cmd = acceleration_cmd;
  _m.accel_cmd.accel = acceleration_cmd;
  ROS_ERROR("-----object_speed: %lf,current_speed: %lf, acc: %lf-------", m_object_speed, current_speed,acceleration_cmd);



  _m.brake_cmd.brake = (int)brake_count;


  #if 1
  if(_m.steer_cmd.steer < -100){
    _m.lamp_cmd.l  = 1;
    _m.lamp_cmd.r = 2;
  }else if(_m.steer_cmd.steer > 100){
    _m.lamp_cmd.l = 2;
    _m.lamp_cmd.r = 1 ;
  }else{
    _m.lamp_cmd.l = 2;
    _m.lamp_cmd.r = 2;
  }
  // setTurnLight(_m.lamp_cmd.l + _m.lamp_cmd.r);
  #endif


  }else{
    speed_pid_controller->Reset();
  }
  set_vehicle_data(_m);

}


void KaiWo::set_vehicle_data(autoware_msgs::VehicleCmd m_data)
{
  // 转换为需要的格式
  setMode(m_data.mode);
  setAccPercent(m_data.accel_cmd.accel);
  setBrakePercent(m_data.brake_cmd.brake);
  setSteerPostion(m_data.steer_cmd.steer);
  setGear(m_data.gear_cmd.gear);
  setTurnLight(m_data.lamp_cmd.l, m_data.lamp_cmd.r);

  m_data.emergency = 2;
  setEPB(m_data.emergency);
  setDoor(door_val);
  
  set_auto_eps();
  set_auto_speed();
  set_auto_ctr_1();
  set_auto_ctr_2();

  // increaseCNT();
  _cnt++;
  ROS_INFO("------------ [can send] mode: %d, acc: %d, brake: %d, angle: %d, gear: %d,turnLight:%d,door: %d", m_vehicle_data.mode, m_vehicle_data.acc, m_vehicle_data.brake, m_vehicle_data.steer, m_vehicle_data.gear, m_vehicle_data.turn_left + m_vehicle_data.turn_right,m_vehicle_data.door);
  ROS_WARN ("------------ [can raw] mode: %d, acc: %d, brake: %d, angle: %f, gear: %d, speed:%f,turnLight:%d",can_info_.devmode,can_info_.drivepedal,can_info_.brakepedal,can_info_.angle,can_info_.driveshift,can_info_.speed,can_info_.light );
}

/////////////////////////////////////////
void KaiWo::set_auto_eps()
{
  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  Auto_EPS_t msg;
  msg.Live_1801B0A0 = _cnt;
  msg.EPS_StrgWhlAngSpd_Cmd = 0 * 2.0;
  msg.EPS_StrgWhlAng_Cmd = m_vehicle_data.steer;
  msg.XOR_Check_1801B0A0 = 0; // calculateCtrlMSGCheckXor(&msg);

  Pack_Auto_EPS_V1(&msg, data, &len, &ide);

  data[7] = calculateCtrlMSGCheckXor(data);

  can_box_publish((int)VehicleCanId::EPS_ID_W,len,data);
}

void KaiWo::set_auto_speed()
{
  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  Auto_Speed_t msg;
  msg.Acce_Cmd = m_vehicle_data.acc;
  msg.EPB_Cmd = m_vehicle_data.epb;
  msg.Gear_Cmd = m_vehicle_data.gear;
  msg.Live_1803B0A0 = _cnt;
  msg.Emergency_Braking_Cmd = 0;  // 相当车身的紧急制动
  msg.XOR_Check_1803B0A0 = 0; // calculateCtrlMSGCheckXor(&msg);

  Pack_Auto_Speed_V1(&msg, data, &len, &ide);
  data[7] = calculateCtrlMSGCheckXor(data);

  can_box_publish((int)VehicleCanId::SPEED_ID_W,len,data);
}

void KaiWo::set_auto_ctr_1()
{

  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  Auto_Ctr_1_t msg;
  msg.AutoDrivingModeRq = m_vehicle_data.mode;
  msg.Door_Cmd = m_vehicle_data.door;             // 1 开, 2关
  msg.Horn_Cmd = 0;
  msg.DaytimeRunningLights_Cmd = 0;
  msg.Left_Turn_Lamps_Cmd = m_vehicle_data.turn_left;
  msg.Right_Turn_Lamps_Cmd = m_vehicle_data.turn_right;
  msg.Warning_Lamps_Cmd = 2;
  msg.Position_Lamps_Cmd = 0;
  msg.Headlight_Cmd = 0;
  msg.Rear_Fog_Lamps_Cmd = 0;
  msg.Left_Front_Fog_Lamps_Cmd = 0;
  msg.Right_Front_Fog_Lamps_Cmd = 0;
  msg.IntCeilingLamp_Cmd = 0;
  msg.IntAtmosphereLamp_Cmd = 0;

  msg.Autopilot_System_State = 1;
  msg.Autopilot_System_Reliability = 2;
  msg.Autopilot_System_Fault_Level = 0;
  msg.Air_Conditioning_Wind_Speed_Cmd = 0;

  msg.Air_Conditioning_Mode_Cmd = 0;
  msg.Air_Conditioning_Set_Tem_Cmd = 0;
  msg.Live_1805B0A0 = _cnt;

  Pack_Auto_Ctr_1_V1(&msg, data, &len, &ide);
  // data[7] = calculateCtrlMSGCheckXor(data);

  can_box_publish((int)VehicleCanId::CTR1_ID_W,len,data);
}


void KaiWo::set_auto_ctr_2()
{
  uint8_t len;
  uint8_t ide;
  uint8_t data[8];

  Auto_Ctr_2_t msg;

  static int clew_tone_count = 0;
  static int clew_tone_type = 0;
  if (clew_tone_flag)
  {
    if(driver_state == FORWARD_STATE){
      clew_tone_type = 3;
    }else if(driver_state == PULL_OVER){
      clew_tone_type = 2;
    }else if(driver_state == FINISH_STATE){
      clew_tone_type = 0xff;
    }

    clew_tone_count = 0;
    clew_tone_flag = false;
  }else if(clew_tone_type == m_vehicle_info.avas){
    clew_tone_type = 0;
  }else if(m_vehicle_info.avas == 3){
    if(clew_tone_count > 500){
      clew_tone_count = 0;
      clew_tone_type = 0xff;
    }
    clew_tone_count++;
  }

  // ROS_ERROR("============================ clew_tone_type: %d,m_vehicle_info.avas: %d,clew_tone_count: %d\n", clew_tone_type,m_vehicle_info.avas ,clew_tone_count);

  msg.AVAS_Cmd = clew_tone_type;                         // 语音控制， 02靠边, 03叮咚
  msg.Autopilot_Request_Remote_Cmd = 0;
  msg.Live_1807B0A0 = _cnt;

  Pack_Auto_Ctr_2_V1(&msg, data, &len, &ide);

  data[7] = calculateCtrlMSGCheckXor(data);

  can_box_publish((int)VehicleCanId::CTR2_ID_W,len,data);
}
////////////////////// SET ////////////////////////////
void KaiWo::increaseCNT() //  计数函数
{
    _cnt++;
    if (_cnt > 15)
    {
        _cnt = 0;
    }
}

uint8_t KaiWo::calculateCtrlMSGCheckSum(void* canFrame) //校验合函数
{
    uint8_t check = 0;
    uint8_t *ptr = (uint8_t *)canFrame;
    for (int i = 0; i < 8; i++)
    {
        check += ptr[i];
    }
    return check;
}

uint8_t KaiWo::calculateCtrlMSGCheckXor(void* canFrame) //校验合函数
{
    uint8_t check = 0;
    uint8_t *ptr = (uint8_t *)canFrame;
    for (int i = 0; i < 8; i++)
    {
        check ^= ptr[i];
    }
    return check;
}


void KaiWo::can_box_publish(int id, int len, uint8_t* data){
#if 0
// autoware_can_msgs::CANPacket canpack;
// canpack.id = (int)VehicleCanId::Gear_Cmd;
// canpack.len = sizeof(data);
// memcpy(&canpack.dat, data, sizeof(data));
// can_box_pub_.publish(canpack);
#else
  autoware_can_msgs::CANPacket canpack;
  canpack.id = id;
  canpack.len = len;
  memcpy(&canpack.dat, data, len);
  can_box_pub_.publish(canpack);
#endif
}

// 设置驾驶模式
void KaiWo::setMode(uint8_t value)
{
  m_vehicle_data.mode = value;

}

// 设置驱动百分比
void KaiWo::setAccPercent(int16_t value)
{
  double tmp = m_acceleration_cmd * 0.1;
  m_vehicle_data.acc = (tmp + 12.8) / 0.1;
}

void KaiWo::setDoor(uint8_t value)
{
  int door_tmp = -1;
  if(value == 1){
        door_tmp = 1;
  }else if(value == 2){
        door_tmp = 0;
  }

  if ((door_tmp != m_vehicle_info.door))
  {
    door_val_flag = false;
    m_vehicle_data.door = value;
  }else{
    m_vehicle_data.door = 0;
  }



  ROS_ERROR("============================ m_vehicle_data.door: %d, m_vehicle_info.door: %d\n", m_vehicle_data.door, m_vehicle_info.door);
}

void KaiWo::setBrakePercent(int16_t value)
{
  m_vehicle_data.brake = value;
}

// 设置制动百分比
void KaiWo::setSteerPostion(int32_t value)
{
  if(steer_val.size() >= 10){
    steer_val.pop_back();                     //删除最后一个元素
  }
  steer_val.insert(steer_val.begin(),value);  // 头部插入
  double mean = std::accumulate(std::begin(steer_val), std::end(steer_val), 0.0) / steer_val.size(); // 得到均值
  // m_vehicle_data.steer = (value + 1080) / 0.1;

  m_vehicle_data.steer = (mean + 1080) / 0.1;
}

// 设置档位
void KaiWo::setGear(uint8_t value)
{
  m_vehicle_data.gear = value;
}

// 设置手刹
void KaiWo::setEPB(uint8_t value)
{
  m_vehicle_data.epb = value;
}

// 设置转向灯
void KaiWo::setTurnLight(uint8_t value_l, uint8_t value_r)
{
  m_vehicle_data.turn_left = value_l;
  m_vehicle_data.turn_right = value_r;
}
// 设置大灯
void KaiWo::setBeamLight(uint8_t value)
{
  m_vehicle_data.beam = value;
}
// 设置喇叭
void KaiWo::setBeep(uint8_t value)
{
  m_vehicle_data.beam = value;
}

////////////////////// GET ////////////////////////////

void KaiWo::vehicle_static(autoware_can_msgs::CANPacket msg)
{
  uint8_t data[8]; //= msg.dat;
  memcpy(data, &msg.dat, sizeof(msg.dat));
  switch (msg.id)
  {
  case VehicleCanId::EPS_ID_R:
    dev_is_ok = true;
    get_vehicle_eps(data);
    break;
  case VehicleCanId::DRIVING_ID_R:
    dev_is_ok = true;
    get_vehicle_driving(data);
    break;
  case VehicleCanId::STATE1_ID_R:
    dev_is_ok = true;
    get_vehicle_state_1(data);
    break;
  case VehicleCanId::STATE5_ID_R:
    dev_is_ok = true;
    get_vehicle_state_5(data);
    break;
  default:
    break;
  }

  get_vehicle_data(m_vehicle_info);
}

bool KaiWo::check_ok(uint8_t *data)
{
    uint8_t ch = data[7];
    data[7] = 0;
    uint8_t ret = calculateCtrlMSGCheckXor(data);
    if (ret != ch)
    {
        return false;
    }
    return true;
}

void KaiWo::run()
{
  can_info_pub_.publish(can_info_);
}
//////////////////////// 3 /////////////////////
void KaiWo::get_vehicle_data(GetVehicleInfo vinfo)
{
  can_info_.devmode = vinfo.mode;
  can_info_.brakepedal = vinfo.brake;
  can_info_.drivepedal = vinfo.acc;
  can_info_.angle = vinfo.steer;
  can_info_.driveshift = vinfo.gear;
  can_info_.light = (vinfo.turn_left + vinfo.turn_right);
  can_info_.speed = vinfo.speed;
}

void KaiWo::get_vehicle_eps(uint8_t *data)
{
  Vehicle_EPS_t msg;
  Unpack_Vehicle_EPS_V1(&msg, data, 0);

  getSteer(msg.Steering_Wheel_Angle);
  // printf("can_raw: get_vehicle_eps ->  %d\n" \
  //                                     ,msg.Steering_Wheel_Angle
  //                                     );
}

void KaiWo::get_vehicle_driving(uint8_t *data)
{
  Vehicle_Driving_t msg;
  Unpack_Vehicle_Driving_V1(&msg, data, 0);

  getAcc(msg.Acce);
  getBrake(msg.Acce);

  getGear(msg.Gear_State);
  getEpb(msg.EPB_State);

  // printf("can_raw: get_vehicle_driving ->  %d, %d, %d\n" \
  //                                     ,msg.Acce \
  //                                     ,msg.Gear_State \
  //                                     ,msg.EPB_State
  //                                     );
}

void KaiWo::get_vehicle_state_1(uint8_t *data)
{
  Vehicle_State_1_t msg;
  Unpack_Vehicle_State_1_V1(&msg,data,0);
  // getMode(msg.DrivingMode_State);
  getSpeed(msg.Vehicle_Speed);
  getTurnLight(msg.Left_Turn_Lamps_State, msg.Right_Turn_Lamps_State);

  getDoor(msg.Door_State);
  getDoorButton(msg.Door_Button_State);




  // printf("can_raw: get_vehicle_state_1 ->  %d, %d, %d, %d, %d\n" \
  //                                     ,msg.DrivingMode_State \
  //                                     ,msg.Vehicle_Speed \
  //                                     ,msg.Left_Turn_Lamps_State \
  //                                     ,msg.Right_Turn_Lamps_State \
  //                                     ,msg.Door_Button_State
  //                                     );
}


void KaiWo::get_vehicle_state_5(uint8_t *data)
{
  Vehicle_State_5_t msg;
  Unpack_Vehicle_State_5_V1(&msg,data,0);

  getAvas(msg.AVAS_State);

  getMode(msg.DrivingMode_State_2);

}
///////////////////////////////////////////////////////
void KaiWo::getMode(uint8_t value)
{
  m_vehicle_info.mode = value;
}

void KaiWo::getAcc(int16_t value)
{
  double valude = (value) * 0.1;
  if (m_vehicle_info.gear == autoware_msgs::Gear::DRIVE)
  {
    if(value >= 0)
      m_vehicle_info.acc = valude;
    else
      m_vehicle_info.acc = 0;
  }
  else if(m_vehicle_info.gear == autoware_msgs::Gear::REVERSE)
  {
    if(value <= 0)
      m_vehicle_info.acc = valude;
    else
      m_vehicle_info.acc = 0;
  }
}

void KaiWo::getBrake(int16_t value)
{

  double valude = (value) * 0.1;
  if (m_vehicle_info.gear == autoware_msgs::Gear::DRIVE)
  {
    if(value >= 0)
      m_vehicle_info.brake = 0;
    else
      m_vehicle_info.brake = valude;
  }
  else if(m_vehicle_info.gear == autoware_msgs::Gear::REVERSE)
  {
    if(value <= 0)
      m_vehicle_info.brake = 0;
    else
      m_vehicle_info.brake = valude;
  }
}

void KaiWo::getSteer(int32_t value)
{
  m_vehicle_info.steer = -(value ) * 0.1;
}

void KaiWo::getEpb(uint8_t value)
{
  m_vehicle_info.epb = value;
}

void KaiWo::getGear(uint8_t value)
{

  switch (value)
  {
  case GearShift::Neutral:
    m_vehicle_info.gear = autoware_msgs::Gear::NEUTRAL;
    break;
  case GearShift::Drive:
    m_vehicle_info.gear = autoware_msgs::Gear::DRIVE;
    break;
  case GearShift::Reverse:
    m_vehicle_info.gear = autoware_msgs::Gear::REVERSE;
    break;
  default:
    m_vehicle_info.gear = autoware_msgs::Gear::NEUTRAL;
    break;
  }
}


void KaiWo::callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg)
{
  // can_info_.speed = msg->twist.linear.x;
  if(fabs(msg->twist.linear.x) - fabs(m_vehicle_info.speed) > 10.0){            // ndt定位的速度与真实速度作对比,判断车是否定位不准
    b_local_status = false;
  }else{
    b_local_status = true;
  }
}

void KaiWo::getSpeed(int16_t value)
{
  m_vehicle_info.speed = (value);   // km/h

  // 发布车辆反馈的速度数据
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.twist.linear.x = can_info_.speed / 3.6; // m/s
  msg.twist.linear.y = can_info_.speed;
  msg.twist.linear.z = m_object_speed;
  msg.twist.angular.x = m_acceleration_cmd;

  double ang_to_steer = (m_vehicle_info.steer / 540) * maxsteerangle;
  msg.twist.angular.z = ang_to_steer ;
  
  
  real_velocity_pub_.publish(msg);
}

void KaiWo::getTurnLight(uint8_t value_l,uint8_t value_r)
{
  m_vehicle_info.turn_left = value_l;
  m_vehicle_info.turn_right = value_r;

#if 0
  static uint8_t old_light=can_info_.light ;
  can_info_.light = value_l;

  if ((value_l == 0) && (old_light !=can_info_.light)){
    pthread_t th;
    if (pthread_create(&th, nullptr, &KaiWo::set_auto_mode, this) != 0)
    {
      std::perror("pthread_create");
      std::exit(1);
    }
  }else if(value_l == 1){
    set_mode = 0;
  }
  old_light = can_info_.light;
#endif

}
void KaiWo::getBeamLight(uint8_t value)
{
  m_vehicle_info.beam = value;
}

void KaiWo::getBeep(uint8_t value)
{
  m_vehicle_info.beep = value;
}


void KaiWo::getAvas(uint8_t value)
{
  m_vehicle_info.avas = value;
}


void KaiWo::getDoor(uint8_t value)
{
  m_vehicle_info.door = value;
  // static int pre_value = value;
  // static int door_count_ = 0;
  // static bool door_flag_ = false;
  // if (value != pre_value && value == 0)
  // {
  //   door_count_ = 0;
  //   door_flag_ = true;
  //   m_door_delay_flag = 1;
  // }else{
  //   if(door_flag_){
  //     if(door_count_ > 150){
  //       door_count_ = 0;
  //       door_flag_ = false;
  //       m_door_delay_flag = 0;
  //     }else{
  //       door_count_++;
  //     }
  //   }
  // }
  // ROS_ERROR("---------------------------value: %d, m_door_delay_flag: %d,door_count_: %d,\n",value, m_door_delay_flag,door_count_);
  // pre_value = value;
  // static int pre_value = value;
  // if (value == 0  && pre_value == 1 && driver_state == FINISH_STATE )
  // {
  //   std_msgs::Bool goal_state;
  //   goal_state.data = true;
  //   goal_on_pub_.publish(goal_state);
  // }

  // pre_value = value;

}

void KaiWo::getDoorButton(uint8_t value)
{
  static bool driver_flag = false;
  if (value == 1 && driver_state == FINISH_STATE)
  {
    driver_flag = true;
  }

  if(driver_state == FORWARD_STATE){
    driver_flag = false;
  }

  std_msgs::Bool goal_state;
  if (driver_flag)
  {
    goal_state.data = true;
  }else{
    goal_state.data = false;
    
  }
  goal_on_pub_.publish(goal_state);

}

// 按下喇叭启动自动模式
void *KaiWo::set_auto_mode(void *arg)
{
#if 0
  ROS_ERROR("------ entry auto car mode ok");
  KaiWo *_this = (KaiWo *)arg;
  if( _this->can_info_.devmode == 0)
  {
    //_this->set_mode = 0;
    //usleep(1000000);
    _this->set_mode = 1;
  }
#endif
}