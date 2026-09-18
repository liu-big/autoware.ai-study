
#include "vehicle/zhongyun_lib/canbase.hpp"

// --------------------------------------------------------------------------
uint32_t Unpack_vehicle_vin_CAN_yeyazhidong_20210(vehicle_vin_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  return vehicle_vin_CANID;
}

uint32_t Pack_vehicle_vin_CAN_yeyazhidong_20210(const vehicle_vin_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < vehicle_vin_DLC; _d[i++] = 0);

  *_len = 8; *_ide = 0;
  return vehicle_vin_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Gear_Shift_Cmd_CAN_yeyazhidong_20210(Gear_Shift_Cmd_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Gear_Enable_control = (_d[0] & (0xFFU));
  _m->Gear_Shift_Req = (_d[1] & (0xFFU));
  _m->IPC_Mode_Shift = (_d[2] & (0xFFU));
  _m->IPC_Stop_Eme = (_d[3] & (0xFFU));
  return Gear_Shift_Cmd_CANID;
}

uint32_t Pack_Gear_Shift_Cmd_CAN_yeyazhidong_20210(const Gear_Shift_Cmd_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Gear_Shift_Cmd_DLC; _d[i++] = 0);

  _d[0] |= (_m->Gear_Enable_control & (0xFFU));
  _d[1] |= (_m->Gear_Shift_Req & (0xFFU));
  _d[2] |= (_m->IPC_Mode_Shift & (0xFFU));
  _d[3] |= (_m->IPC_Stop_Eme & (0xFFU));
  *_len = 8; *_ide = 0;
  return Gear_Shift_Cmd_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_steering_control_CAN_yeyazhidong_20210(steering_control_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Steering_Enable_control = (_d[0] & (0xFFU));
  _m->Steering_Pos_Req = (((_d[2] & (0xFFU)) << 8) | (_d[1] & (0xFFU))) - 32767;
  return steering_control_CANID;
}

uint32_t Pack_steering_control_CAN_yeyazhidong_20210(const steering_control_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < steering_control_DLC; _d[i++] = 0);

  _d[0] |= (_m->Steering_Enable_control & (0xFFU));
  _d[1] |= (_m->Steering_Pos_Req & (0xFFU));
  _d[2] |= ((_m->Steering_Pos_Req >> 8) & (0xFFU));
  *_len = 8; *_ide = 0;
  return steering_control_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Drive_control_CAN_yeyazhidong_20210(Drive_control_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Driven_Enable_control = (_d[0] & (0xFFU));
  _m->Drive_Tq_Req = ((_d[2] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
  return Drive_control_CANID;
}

uint32_t Pack_Drive_control_CAN_yeyazhidong_20210(const Drive_control_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Drive_control_DLC; _d[i++] = 0);
  _d[0] |= (_m->Driven_Enable_control & (0xFFU));
  _d[1] |= (_m->Drive_Tq_Req & (0xFFU));
  _d[2] |= ((_m->Drive_Tq_Req >> 8) & (0xFFU));
  *_len = 8; *_ide = 0;
  return Drive_control_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_brake_control_CAN_yeyazhidong_20210(brake_control_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Brake_Enable_control = (_d[0] & (0xFFU));
  _m->Brake_Tq_Req = ((_d[2] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
  return brake_control_CANID;
}

uint32_t Pack_brake_control_CAN_yeyazhidong_20210(const brake_control_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < brake_control_DLC; _d[i++] = 0);

  _d[0] |= (_m->Brake_Enable_control & (0xFFU));
  _d[1] |= (_m->Brake_Tq_Req & (0xFFU));
  _d[2] |= ((_m->Brake_Tq_Req >> 8) & (0xFFU));
  *_len = 8; *_ide = 0;
  return brake_control_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Parking_control_CAN_yeyazhidong_20210(Parking_control_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Parking_Enable_control = (_d[0] & (0xFFU));
  _m->Parking_Cmd = (_d[1] & (0xFFU));
  return Parking_control_CANID;
}

uint32_t Pack_Parking_control_CAN_yeyazhidong_20210(const Parking_control_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Parking_control_DLC; _d[i++] = 0);

  _d[0] |= (_m->Parking_Enable_control & (0xFFU));
  _d[1] |= (_m->Parking_Cmd & (0xFFU));
  *_len = 8; *_ide = 0;
  return Parking_control_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_VCU_FeedBack_CAN_yeyazhidong_20210(VCU_FeedBack_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Vehicle_Spd = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->Steering_Ang = (((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU))) - 32767;
  _m->Gear_Pos = (_d[4] & (0xFFU));
  _m->Brake_Tq = (_d[5] & (0xFFU));
  _m->Brake_pedal_Position = (_d[6] & (0xFFU));
  _m->CheckSum_Byte = (_d[7] & (0xFFU));
  return VCU_FeedBack_CANID;
}

uint32_t Pack_VCU_FeedBack_CAN_yeyazhidong_20210(const VCU_FeedBack_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < VCU_FeedBack_DLC; _d[i++] = 0);

  _d[0] |= (_m->Vehicle_Spd & (0xFFU));
  _d[1] |= ((_m->Vehicle_Spd >> 8) & (0xFFU));
  _d[2] |= (_m->Steering_Ang & (0xFFU));
  _d[3] |= ((_m->Steering_Ang >> 8) & (0xFFU));
  _d[4] |= (_m->Gear_Pos & (0xFFU));
  _d[5] |= (_m->Brake_Tq & (0xFFU));
  _d[6] |= (_m->Brake_pedal_Position & (0xFFU));
  _d[7] |= (_m->CheckSum_Byte & (0xFFU));
  *_len = 8; *_ide = 0;
  return VCU_FeedBack_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_BMS_FeedBack_CAN_yeyazhidong_20210(BMS_FeedBack_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Soc = (_d[0] & (0xFFU));
  _m->Vol = ((_d[2] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
  _m->Current = (_d[3] & (0xFFU));
  _m->CheckSum_Byte1 = (_d[7] & (0xFFU));
  return BMS_FeedBack_CANID;
}

uint32_t Pack_BMS_FeedBack_CAN_yeyazhidong_20210(const BMS_FeedBack_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < BMS_FeedBack_DLC; _d[i++] = 0);

  _d[0] |= (_m->Soc & (0xFFU));
  _d[1] |= (_m->Vol & (0xFFU));
  _d[2] |= ((_m->Vol >> 8) & (0xFFU));
  _d[3] |= (_m->Current & (0xFFU));
  _d[7] |= (_m->CheckSum_Byte1 & (0xFFU));
  *_len = 8; *_ide = 0;
  return BMS_FeedBack_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Enable_Fbk_CAN_yeyazhidong_20210(Enable_Fbk_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Gear_Enb_fbk = (_d[0] & (0xFFU));
  _m->Steering_Enb_fbk = (_d[1] & (0xFFU));
  _m->Drive_Enb_fbk = (_d[2] & (0xFFU));
  _m->Brake_Enb_fbk = (_d[3] & (0xFFU));
  _m->Parking_Enb_fbk = (_d[4] & (0xFFU));
  _m->RC_Takerover_Flg = (_d[5] & (0xFFU));
  _m->CheckSum_Byte2 = (_d[7] & (0xFFU));
  return Enable_Fbk_CANID;
}

uint32_t Pack_Enable_Fbk_CAN_yeyazhidong_20210(const Enable_Fbk_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Enable_Fbk_DLC; _d[i++] = 0);

  _d[0] |= (_m->Gear_Enb_fbk & (0xFFU));
  _d[1] |= (_m->Steering_Enb_fbk & (0xFFU));
  _d[2] |= (_m->Drive_Enb_fbk & (0xFFU));
  _d[3] |= (_m->Brake_Enb_fbk & (0xFFU));
  _d[4] |= (_m->Parking_Enb_fbk & (0xFFU));
  _d[5] |= (_m->RC_Takerover_Flg & (0xFFU));
  _d[7] |= (_m->CheckSum_Byte2 & (0xFFU));
  *_len = 8; *_ide = 0;
  return Enable_Fbk_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_FourWheel_Spd_Fbk_CAN_yeyazhidong_20210(FourWheel_Spd_Fbk_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Fl_Spd_Valid = (_d[0] & (0xFFU));
  _m->Fl_Wheel_Spd = (_d[1] & (0xFFU));
  _m->Rl_Spd_Valid = (_d[2] & (0xFFU));
  _m->Rl_Wheel_Spd = (_d[3] & (0xFFU));
  _m->Fr_Spd_Valid = (_d[4] & (0xFFU));
  _m->Fr_Wheel_Spd = (_d[5] & (0xFFU));
  _m->Rr_Spd_Valid = (_d[6] & (0xFFU));
  _m->Rr_Wheel_spd = (_d[7] & (0xFFU));
  return FourWheel_Spd_Fbk_CANID;
}

uint32_t Pack_FourWheel_Spd_Fbk_CAN_yeyazhidong_20210(const FourWheel_Spd_Fbk_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < FourWheel_Spd_Fbk_DLC; _d[i++] = 0);

  _d[0] |= (_m->Fl_Spd_Valid & (0xFFU));
  _d[1] |= (_m->Fl_Wheel_Spd & (0xFFU));
  _d[2] |= (_m->Rl_Spd_Valid & (0xFFU));
  _d[3] |= (_m->Rl_Wheel_Spd & (0xFFU));
  _d[4] |= (_m->Fr_Spd_Valid & (0xFFU));
  _d[5] |= (_m->Fr_Wheel_Spd & (0xFFU));
  _d[6] |= (_m->Rr_Spd_Valid & (0xFFU));
  _d[7] |= (_m->Rr_Wheel_spd & (0xFFU));
  *_len = 8; *_ide = 0;
  return FourWheel_Spd_Fbk_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Motor_Spd_Fbk_CAN_yeyazhidong_20210(Motor_Spd_Fbk_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Motor_spd = (((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU))) - 10000;
  _m->Rcontrol_Stop_Emcy = (_d[2] & (0xFFU));
  return Motor_Spd_Fbk_CANID;
}

uint32_t Pack_Motor_Spd_Fbk_CAN_yeyazhidong_20210(const Motor_Spd_Fbk_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Motor_Spd_Fbk_DLC; _d[i++] = 0);

  _d[0] |= (_m->Motor_spd & (0xFFU));
  _d[1] |= ((_m->Motor_spd >> 8) & (0xFFU));
  _d[2] |= (_m->Rcontrol_Stop_Emcy & (0xFFU));
  *_len = 8; *_ide = 0;
  return Motor_Spd_Fbk_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Err_Fbk_CAN_yeyazhidong_20210(Err_Fbk_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Error_code = (_d[0] & (0xFFU));
  _m->Code_Num = (_d[1] & (0xFFU));
  _m->Brake_Err = (_d[2] & (0xFFU));
  _m->CheckSum_Byte3 = (_d[7] & (0xFFU));
  return Err_Fbk_CANID;
}

uint32_t Pack_Err_Fbk_CAN_yeyazhidong_20210(const Err_Fbk_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Err_Fbk_DLC; _d[i++] = 0);

  _d[0] |= (_m->Error_code & (0xFFU));
  _d[1] |= (_m->Code_Num & (0xFFU));
  _d[2] |= (_m->Brake_Err & (0xFFU));
  _d[7] |= (_m->CheckSum_Byte3 & (0xFFU));
  *_len = 8; *_ide = 0;
  return Err_Fbk_CANID;
}

