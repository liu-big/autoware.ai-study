
// this file were genereted by coderdbc.com web service
// any questions - mailto:coderdbc@gmail.com

#include "vehicle/kaiwo_lib/canbase.hpp"

// --------------------------------------------------------------------------
uint32_t Unpack_WheelSpd_V1(WheelSpd_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->FLWheelSpd = ((_d[1] & (0x03U)) << 8) | (_d[0] & (0xFFU));
  _m->FRWheelSpd = ((_d[2] & (0x0FU)) << 6) | ((_d[1] >> 2) & (0x3FU));
  _m->RLWheelSpd = ((_d[3] & (0x3FU)) << 4) | ((_d[2] >> 4) & (0x0FU));
  _m->RRWheelSpd = ((_d[4] & (0xFFU)) << 2) | ((_d[3] >> 6) & (0x03U));
  _m->FLWheelSpd_Vd = (_d[5] & (0x01U));
  _m->FRWheelSpd_Vd = ((_d[5] >> 1) & (0x01U));
  _m->RLWheelSpd_Vd = ((_d[5] >> 2) & (0x01U));
  _m->RRWheelSpd_Vd = ((_d[5] >> 3) & (0x01U));
  _m->ABS_Status = ((_d[5] >> 4) & (0x03U));
  _m->FLWheelDrvDirection = (_d[6] & (0x03U));
  _m->FRWheelDrvDirection = ((_d[6] >> 2) & (0x03U));
  _m->RLWheelDrvDirection = ((_d[6] >> 4) & (0x03U));
  _m->RRWheelDrvDirection = ((_d[6] >> 6) & (0x03U));
  return WheelSpd_CANID;
}

uint32_t Pack_WheelSpd_V1(const WheelSpd_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < WheelSpd_DLC; _d[i++] = 0);

  _d[0] |= (_m->FLWheelSpd & (0xFFU));
  _d[1] |= ((_m->FLWheelSpd >> 8) & (0x03U)) | ((_m->FRWheelSpd & (0x3FU)) << 2);
  _d[2] |= ((_m->FRWheelSpd >> 6) & (0x0FU)) | ((_m->RLWheelSpd & (0x0FU)) << 4);
  _d[3] |= ((_m->RLWheelSpd >> 4) & (0x3FU)) | ((_m->RRWheelSpd & (0x03U)) << 6);
  _d[4] |= ((_m->RRWheelSpd >> 2) & (0xFFU));
  _d[5] |= (_m->FLWheelSpd_Vd & (0x01U)) | ((_m->FRWheelSpd_Vd & (0x01U)) << 1) | ((_m->RLWheelSpd_Vd & (0x01U)) << 2) | ((_m->RRWheelSpd_Vd & (0x01U)) << 3) | ((_m->ABS_Status & (0x03U)) << 4);
  _d[6] |= (_m->FLWheelDrvDirection & (0x03U)) | ((_m->FRWheelDrvDirection & (0x03U)) << 2) | ((_m->RLWheelDrvDirection & (0x03U)) << 4) | ((_m->RRWheelDrvDirection & (0x03U)) << 6);
  *_len = 8; *_ide = 1;
  return WheelSpd_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Tire_Press_V1(Tire_Press_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->LFTirePress = (_d[0] & (0x7FU));
  _m->LFTirePressVd = ((_d[0] >> 7) & (0x01U));
  _m->RFTirePress = (_d[1] & (0x7FU));
  _m->RFTirePressVd = ((_d[1] >> 7) & (0x01U));
  _m->LRTirePress = (_d[2] & (0x7FU));
  _m->LRTirePressVd = ((_d[2] >> 7) & (0x01U));
  _m->RRTirePress = (_d[3] & (0x7FU));
  _m->RRTirePressVd = ((_d[3] >> 7) & (0x01U));
  _m->TirePressAlarm = (_d[4] & (0x03U));
  return Tire_Press_CANID;
}

uint32_t Pack_Tire_Press_V1(const Tire_Press_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Tire_Press_DLC; _d[i++] = 0);

  _d[0] |= (_m->LFTirePress & (0x7FU)) | ((_m->LFTirePressVd & (0x01U)) << 7);
  _d[1] |= (_m->RFTirePress & (0x7FU)) | ((_m->RFTirePressVd & (0x01U)) << 7);
  _d[2] |= (_m->LRTirePress & (0x7FU)) | ((_m->LRTirePressVd & (0x01U)) << 7);
  _d[3] |= (_m->RRTirePress & (0x7FU)) | ((_m->RRTirePressVd & (0x01U)) << 7);
  _d[4] |= (_m->TirePressAlarm & (0x03U));
  *_len = 8; *_ide = 1;
  return Tire_Press_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Tire_Temp_V1(Tire_Temp_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->LFTireTemp = ((_d[0] & (0x7FU))) - 40;
  _m->LFTireTempVd = ((_d[0] >> 7) & (0x01U));
  _m->RFTireTemp = ((_d[1] & (0x7FU))) - 40;
  _m->RFTireTempVd = ((_d[1] >> 7) & (0x01U));
  _m->LRTireTemp = ((_d[2] & (0x7FU))) - 40;
  _m->LRTireTempVd = ((_d[2] >> 7) & (0x01U));
  _m->RRTireTemp = ((_d[3] & (0x7FU))) - 40;
  _m->RRTireTempVd = ((_d[3] >> 7) & (0x01U));
  return Tire_Temp_CANID;
}

uint32_t Pack_Tire_Temp_V1(const Tire_Temp_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Tire_Temp_DLC; _d[i++] = 0);

  _d[0] |= (_m->LFTireTemp & (0x7FU)) | ((_m->LFTireTempVd & (0x01U)) << 7);
  _d[1] |= (_m->RFTireTemp & (0x7FU)) | ((_m->RFTireTempVd & (0x01U)) << 7);
  _d[2] |= (_m->LRTireTemp & (0x7FU)) | ((_m->LRTireTempVd & (0x01U)) << 7);
  _d[3] |= (_m->RRTireTemp & (0x7FU)) | ((_m->RRTireTempVd & (0x01U)) << 7);
  *_len = 8; *_ide = 1;
  return Tire_Temp_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_UW_Sensor_Rear_V1(UW_Sensor_Rear_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Rear_Left_Sensor_Dis = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->Rear_Left_Center_Sensor_Dis = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->Rear_Right_Center_Sensor_Dis = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  _m->Rear_Right_Sensor_Dis = ((_d[7] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
  return UW_Sensor_Rear_CANID;
}

uint32_t Pack_UW_Sensor_Rear_V1(const UW_Sensor_Rear_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < UW_Sensor_Rear_DLC; _d[i++] = 0);

  _d[0] |= (_m->Rear_Left_Sensor_Dis & (0xFFU));
  _d[1] |= ((_m->Rear_Left_Sensor_Dis >> 8) & (0xFFU));
  _d[2] |= (_m->Rear_Left_Center_Sensor_Dis & (0xFFU));
  _d[3] |= ((_m->Rear_Left_Center_Sensor_Dis >> 8) & (0xFFU));
  _d[4] |= (_m->Rear_Right_Center_Sensor_Dis & (0xFFU));
  _d[5] |= ((_m->Rear_Right_Center_Sensor_Dis >> 8) & (0xFFU));
  _d[6] |= (_m->Rear_Right_Sensor_Dis & (0xFFU));
  _d[7] |= ((_m->Rear_Right_Sensor_Dis >> 8) & (0xFFU));
  *_len = 8; *_ide = 1;
  return UW_Sensor_Rear_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_UW_Sensor_Front_V1(UW_Sensor_Front_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Front_Left_Sensor_Dis = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->Front_Left_Center_Sensor_Dis = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->Front_Right_Center_Sensor_Dis = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  _m->Front_Right_Sensor_Dis = ((_d[7] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
  return UW_Sensor_Front_CANID;
}

uint32_t Pack_UW_Sensor_Front_V1(const UW_Sensor_Front_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < UW_Sensor_Front_DLC; _d[i++] = 0);

  _d[0] |= (_m->Front_Left_Sensor_Dis & (0xFFU));
  _d[1] |= ((_m->Front_Left_Sensor_Dis >> 8) & (0xFFU));
  _d[2] |= (_m->Front_Left_Center_Sensor_Dis & (0xFFU));
  _d[3] |= ((_m->Front_Left_Center_Sensor_Dis >> 8) & (0xFFU));
  _d[4] |= (_m->Front_Right_Center_Sensor_Dis & (0xFFU));
  _d[5] |= ((_m->Front_Right_Center_Sensor_Dis >> 8) & (0xFFU));
  _d[6] |= (_m->Front_Right_Sensor_Dis & (0xFFU));
  _d[7] |= ((_m->Front_Right_Sensor_Dis >> 8) & (0xFFU));
  *_len = 8; *_ide = 1;
  return UW_Sensor_Front_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_UW_Sensor_Side_V1(UW_Sensor_Side_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Left_Front_Sensor_Dis = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->Left_Rear_Sensor_Dis = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->Right_Front_Sensor_Dis = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  _m->Right_Rear_Sensor_Dis = ((_d[7] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
  return UW_Sensor_Side_CANID;
}

uint32_t Pack_UW_Sensor_Side_V1(const UW_Sensor_Side_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < UW_Sensor_Side_DLC; _d[i++] = 0);

  _d[0] |= (_m->Left_Front_Sensor_Dis & (0xFFU));
  _d[1] |= ((_m->Left_Front_Sensor_Dis >> 8) & (0xFFU));
  _d[2] |= (_m->Left_Rear_Sensor_Dis & (0xFFU));
  _d[3] |= ((_m->Left_Rear_Sensor_Dis >> 8) & (0xFFU));
  _d[4] |= (_m->Right_Front_Sensor_Dis & (0xFFU));
  _d[5] |= ((_m->Right_Front_Sensor_Dis >> 8) & (0xFFU));
  _d[6] |= (_m->Right_Rear_Sensor_Dis & (0xFFU));
  _d[7] |= ((_m->Right_Rear_Sensor_Dis >> 8) & (0xFFU));
  *_len = 8; *_ide = 1;
  return UW_Sensor_Side_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_UW_Sensor_Fault_V1(UW_Sensor_Fault_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Rear_Left_Sensor_Err = (_d[4] & (0x01U));
  _m->Rear_Left_Center_Sensor_Err = ((_d[4] >> 1) & (0x01U));
  _m->Rear_Right_Center_Sensor_Err = ((_d[4] >> 2) & (0x01U));
  _m->Rear_Right_Sensor_Err = ((_d[4] >> 3) & (0x01U));
  _m->Front_Left_Sensor_Err = ((_d[4] >> 4) & (0x01U));
  _m->Front_Left_Center_Sensor_Err = ((_d[4] >> 5) & (0x01U));
  _m->Front_Right_Center_Sensor_Err = ((_d[4] >> 6) & (0x01U));
  _m->Front_Right_Sensor_Err = ((_d[4] >> 7) & (0x01U));
  _m->Left_Front_Sensor_Err = (_d[5] & (0x01U));
  _m->Left_Rear_Sensor_Err = ((_d[5] >> 1) & (0x01U));
  _m->Right_Front_Sensor_Err = ((_d[5] >> 2) & (0x01U));
  _m->Right_Rear_Sensor_Err = ((_d[5] >> 3) & (0x01U));
  _m->Live_0CFF0523 = (_d[6] & (0xFFU));
  return UW_Sensor_Fault_CANID;
}

uint32_t Pack_UW_Sensor_Fault_V1(const UW_Sensor_Fault_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < UW_Sensor_Fault_DLC; _d[i++] = 0);

  _d[4] |= (_m->Rear_Left_Sensor_Err & (0x01U)) | ((_m->Rear_Left_Center_Sensor_Err & (0x01U)) << 1) | ((_m->Rear_Right_Center_Sensor_Err & (0x01U)) << 2) | ((_m->Rear_Right_Sensor_Err & (0x01U)) << 3) | ((_m->Front_Left_Sensor_Err & (0x01U)) << 4) | ((_m->Front_Left_Center_Sensor_Err & (0x01U)) << 5) | ((_m->Front_Right_Center_Sensor_Err & (0x01U)) << 6) | ((_m->Front_Right_Sensor_Err & (0x01U)) << 7);
  _d[5] |= (_m->Left_Front_Sensor_Err & (0x01U)) | ((_m->Left_Rear_Sensor_Err & (0x01U)) << 1) | ((_m->Right_Front_Sensor_Err & (0x01U)) << 2) | ((_m->Right_Rear_Sensor_Err & (0x01U)) << 3);
  _d[6] |= (_m->Live_0CFF0523 & (0xFFU));
  *_len = 8; *_ide = 1;
  return UW_Sensor_Fault_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Auto_EPS_V1(Auto_EPS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Live_1801B0A0 = (_d[1] & (0xFFU));
  _m->EPS_StrgWhlAngSpd_Cmd = (_d[2] & (0xFFU));
  _m->EPS_StrgWhlAng_Cmd = (((_d[4] & (0xFFU)) << 8) | (_d[3] & (0xFFU))) - 10800;
  _m->XOR_Check_1801B0A0 = (_d[7] & (0xFFU));
  return Auto_EPS_CANID;
}

uint32_t Pack_Auto_EPS_V1(const Auto_EPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Auto_EPS_DLC; _d[i++] = 0);

  _d[1] |= (_m->Live_1801B0A0 & (0xFFU));
  _d[2] |= (_m->EPS_StrgWhlAngSpd_Cmd & (0xFFU));
  _d[3] |= (_m->EPS_StrgWhlAng_Cmd & (0xFFU));
  _d[4] |= ((_m->EPS_StrgWhlAng_Cmd >> 8) & (0xFFU));
  _d[7] |= (_m->XOR_Check_1801B0A0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Auto_EPS_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Remote_EPS_V1(Remote_EPS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Live_1801B0C0 = (_d[1] & (0xFFU));
  _m->EPS_StrgWhlAngSpd_Cmd = (_d[2] & (0xFFU));
  _m->EPS_StrgWhlAng_Cmd = (((_d[4] & (0xFFU)) << 8) | (_d[3] & (0xFFU))) - 10800;
  _m->XOR_Check_1801B0C0 = (_d[7] & (0xFFU));
  return Remote_EPS_CANID;
}

uint32_t Pack_Remote_EPS_V1(const Remote_EPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Remote_EPS_DLC; _d[i++] = 0);

  _d[1] |= (_m->Live_1801B0C0 & (0xFFU));
  _d[2] |= (_m->EPS_StrgWhlAngSpd_Cmd & (0xFFU));
  _d[3] |= (_m->EPS_StrgWhlAng_Cmd & (0xFFU));
  _d[4] |= ((_m->EPS_StrgWhlAng_Cmd >> 8) & (0xFFU));
  _d[7] |= (_m->XOR_Check_1801B0C0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Remote_EPS_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_EPS_V1(Vehicle_EPS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->EPS_Control_State = (_d[0] & (0xFFU));
  _m->Steering_Wheel_Torque = ((_d[1] & (0xFFU))) - 128;
  _m->EPS_Output_Torque = ((_d[2] & (0xFFU))) - 128;
  _m->Steering_Wheel_Angle = (((_d[4] & (0xFFU)) << 8) | (_d[3] & (0xFFU))) - 10800;
  _m->EPS_Controller_Temperature = (_d[5] & (0x3FU));
  _m->EPS_StrgWhlAngSnsrCalSts = ((_d[5] >> 6) & (0x01U));
  _m->EPS_DriverInterventionSteering = ((_d[5] >> 7) & (0x01U));
  _m->EPS_Fault_Level = (_d[6] & (0x03U));
  _m->Live_1802A0B0 = (_d[7] & (0xFFU));
  return Vehicle_EPS_CANID;
}

uint32_t Pack_Vehicle_EPS_V1(const Vehicle_EPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_EPS_DLC; _d[i++] = 0);

  _d[0] |= (_m->EPS_Control_State & (0xFFU));
  _d[1] |= (_m->Steering_Wheel_Torque & (0xFFU));
  _d[2] |= (_m->EPS_Output_Torque & (0xFFU));
  _d[3] |= (_m->Steering_Wheel_Angle & (0xFFU));
  _d[4] |= ((_m->Steering_Wheel_Angle >> 8) & (0xFFU));
  _d[5] |= (_m->EPS_Controller_Temperature & (0x3FU)) | ((_m->EPS_StrgWhlAngSnsrCalSts & (0x01U)) << 6) | ((_m->EPS_DriverInterventionSteering & (0x01U)) << 7);
  _d[6] |= (_m->EPS_Fault_Level & (0x03U));
  _d[7] |= (_m->Live_1802A0B0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Vehicle_EPS_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Auto_Speed_V1(Auto_Speed_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Acce_Cmd = ((_d[0] & (0xFFU))) - 128;
  _m->EPB_Cmd = ((_d[1] >> 2) & (0x03U));
  _m->Gear_Cmd = ((_d[1] >> 4) & (0x0FU));
  _m->Live_1803B0A0 = (_d[2] & (0xFFU));
  _m->Emergency_Braking_Cmd = (_d[3] & (0x01U));
  _m->XOR_Check_1803B0A0 = (_d[7] & (0xFFU));
  return Auto_Speed_CANID;
}

uint32_t Pack_Auto_Speed_V1(const Auto_Speed_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Auto_Speed_DLC; _d[i++] = 0);

  _d[0] |= (_m->Acce_Cmd & (0xFFU));
  _d[1] |= ((_m->EPB_Cmd & (0x03U)) << 2) | ((_m->Gear_Cmd & (0x0FU)) << 4);
  _d[2] |= (_m->Live_1803B0A0 & (0xFFU));
  _d[3] |= (_m->Emergency_Braking_Cmd & (0x01U));
  _d[7] |= (_m->XOR_Check_1803B0A0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Auto_Speed_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Remote_Speed_V1(Remote_Speed_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Acce_Cmd = ((_d[0] & (0xFFU))) - 128;
  _m->EPB_Cmd = ((_d[1] >> 2) & (0x03U));
  _m->Gear_Cmd = ((_d[1] >> 4) & (0x0FU));
  _m->Live_1803B0C0 = (_d[2] & (0xFFU));
  _m->Emergency_Barking_Cmd = (_d[3] & (0x01U));
  _m->XOR_Check_1803B0C0 = (_d[7] & (0xFFU));
  return Remote_Speed_CANID;
}

uint32_t Pack_Remote_Speed_V1(const Remote_Speed_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Remote_Speed_DLC; _d[i++] = 0);

  _d[0] |= (_m->Acce_Cmd & (0xFFU));
  _d[1] |= ((_m->EPB_Cmd & (0x03U)) << 2) | ((_m->Gear_Cmd & (0x0FU)) << 4);
  _d[2] |= (_m->Live_1803B0C0 & (0xFFU));
  _d[3] |= (_m->Emergency_Barking_Cmd & (0x01U));
  _d[7] |= (_m->XOR_Check_1803B0C0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Remote_Speed_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_Driving_V1(Vehicle_Driving_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->EPB_State = (_d[0] & (0x03U));
  _m->Gear_State = ((_d[0] >> 2) & (0x0FU));
  _m->Emergency_Barking_State = ((_d[0] >> 6) & (0x01U));
  _m->Ultrasonic_Radar_Barking_State = ((_d[0] >> 7) & (0x01U));
  _m->Motor_Speed = (((_d[2] & (0xFFU)) << 8) | (_d[1] & (0xFFU))) - 15000;
  _m->Motor_Torque = (((_d[4] & (0xFFU)) << 8) | (_d[3] & (0xFFU))) - 5000;
  _m->Motor_Speed_Vehicle_Speed_Ratio = ((_d[5] & (0xFFU))) + 30;
  _m->Acce = ((_d[6] & (0xFFU))) - 128;
  _m->XOR_Check_1804A0B0 = (_d[7] & (0xFFU));
  return Vehicle_Driving_CANID;
}

uint32_t Pack_Vehicle_Driving_V1(const Vehicle_Driving_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_Driving_DLC; _d[i++] = 0);

  _d[0] |= (_m->EPB_State & (0x03U)) | ((_m->Gear_State & (0x0FU)) << 2) | ((_m->Emergency_Barking_State & (0x01U)) << 6) | ((_m->Ultrasonic_Radar_Barking_State & (0x01U)) << 7);
  _d[1] |= (_m->Motor_Speed & (0xFFU));
  _d[2] |= ((_m->Motor_Speed >> 8) & (0xFFU));
  _d[3] |= (_m->Motor_Torque & (0xFFU));
  _d[4] |= ((_m->Motor_Torque >> 8) & (0xFFU));
  _d[5] |= (_m->Motor_Speed_Vehicle_Speed_Ratio & (0xFFU));
  _d[6] |= (_m->Acce & (0xFFU));
  _d[7] |= (_m->XOR_Check_1804A0B0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Vehicle_Driving_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Auto_Ctr_1_V1(Auto_Ctr_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AutoDrivingModeRq = (_d[0] & (0x03U));
  _m->Door_Cmd = ((_d[0] >> 2) & (0x03U));
  _m->Horn_Cmd = ((_d[0] >> 4) & (0x03U));
  _m->DaytimeRunningLights_Cmd = ((_d[0] >> 6) & (0x03U));
  _m->Left_Turn_Lamps_Cmd = (_d[1] & (0x03U));
  _m->Right_Turn_Lamps_Cmd = ((_d[1] >> 2) & (0x03U));
  _m->Warning_Lamps_Cmd = ((_d[1] >> 4) & (0x03U));
  _m->Position_Lamps_Cmd = ((_d[1] >> 6) & (0x03U));
  _m->Headlight_Cmd = (_d[2] & (0x03U));
  _m->Rear_Fog_Lamps_Cmd = ((_d[2] >> 2) & (0x03U));
  _m->Left_Front_Fog_Lamps_Cmd = ((_d[2] >> 4) & (0x03U));
  _m->Right_Front_Fog_Lamps_Cmd = ((_d[2] >> 6) & (0x03U));
  _m->IntCeilingLamp_Cmd = (_d[3] & (0x03U));
  _m->IntAtmosphereLamp_Cmd = ((_d[3] >> 2) & (0x03U));
  _m->Autopilot_System_State = (_d[4] & (0x03U));
  _m->Autopilot_System_Reliability = ((_d[4] >> 2) & (0x03U));
  _m->Autopilot_System_Fault_Level = ((_d[4] >> 4) & (0x03U));
  _m->Air_Conditioning_Wind_Speed_Cmd = ((_d[4] >> 6) & (0x03U));
  _m->Air_Conditioning_Mode_Cmd = (_d[5] & (0x03U));
  _m->Air_Conditioning_Set_Tem_Cmd = (((_d[5] >> 3) & (0x1FU))) + 32;
  _m->Live_1805B0A0 = (_d[7] & (0xFFU));
  return Auto_Ctr_1_CANID;
}

uint32_t Pack_Auto_Ctr_1_V1(const Auto_Ctr_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Auto_Ctr_1_DLC; _d[i++] = 0);

  _d[0] |= (_m->AutoDrivingModeRq & (0x03U)) | ((_m->Door_Cmd & (0x03U)) << 2) | ((_m->Horn_Cmd & (0x03U)) << 4) | ((_m->DaytimeRunningLights_Cmd & (0x03U)) << 6);
  _d[1] |= (_m->Left_Turn_Lamps_Cmd & (0x03U)) | ((_m->Right_Turn_Lamps_Cmd & (0x03U)) << 2) | ((_m->Warning_Lamps_Cmd & (0x03U)) << 4) | ((_m->Position_Lamps_Cmd & (0x03U)) << 6);
  _d[2] |= (_m->Headlight_Cmd & (0x03U)) | ((_m->Rear_Fog_Lamps_Cmd & (0x03U)) << 2) | ((_m->Left_Front_Fog_Lamps_Cmd & (0x03U)) << 4) | ((_m->Right_Front_Fog_Lamps_Cmd & (0x03U)) << 6);
  _d[3] |= (_m->IntCeilingLamp_Cmd & (0x03U)) | ((_m->IntAtmosphereLamp_Cmd & (0x03U)) << 2);
  _d[4] |= (_m->Autopilot_System_State & (0x03U)) | ((_m->Autopilot_System_Reliability & (0x03U)) << 2) | ((_m->Autopilot_System_Fault_Level & (0x03U)) << 4) | ((_m->Air_Conditioning_Wind_Speed_Cmd & (0x03U)) << 6);
  _d[5] |= (_m->Air_Conditioning_Mode_Cmd & (0x03U)) | ((_m->Air_Conditioning_Set_Tem_Cmd & (0x1FU)) << 3);
  _d[7] |= (_m->Live_1805B0A0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Auto_Ctr_1_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Remote_Ctr_1_V1(Remote_Ctr_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->RemoteDrivingModeRq = (_d[0] & (0x03U));
  _m->Door_Cmd = ((_d[0] >> 2) & (0x03U));
  _m->Horn_Cmd = ((_d[0] >> 4) & (0x03U));
  _m->DaytimeRunningLights_Cmd = ((_d[0] >> 6) & (0x03U));
  _m->Left_Turn_Lamps_Cmd = (_d[1] & (0x03U));
  _m->Right_Turn_Lamps_Cmd = ((_d[1] >> 2) & (0x03U));
  _m->Warning_Lamps_Cmd = ((_d[1] >> 4) & (0x03U));
  _m->Positing_Lamps_Cmd = ((_d[1] >> 6) & (0x03U));
  _m->Headlight_Cmd = (_d[2] & (0x03U));
  _m->Rear_Fog_Lamps_Cmd = ((_d[2] >> 2) & (0x03U));
  _m->Left_Front_Fog_Lamps_Cmd = ((_d[2] >> 4) & (0x03U));
  _m->Right_Front_Fog_Lamps_Cmd = ((_d[2] >> 6) & (0x03U));
  _m->IntCeilingLamp_Cmd = (_d[3] & (0x03U));
  _m->IntAtmosphereLamp_Cmd = ((_d[3] >> 2) & (0x03U));
  _m->Remote_System_Connection_Cmd = (_d[4] & (0x03U));
  _m->Remote_System_Reliability = ((_d[4] >> 2) & (0x03U));
  _m->Remote_System_Fault_Level = ((_d[4] >> 4) & (0x03U));
  _m->Air_Conditioning_Wind_Speed_Cmd = ((_d[4] >> 6) & (0x03U));
  _m->Air_Conditioning_Mode_Cmd = (_d[5] & (0x07U));
  _m->Air_Conditioning_Set_Tem_Cmd = (((_d[5] >> 3) & (0x1FU))) + 32;
  _m->Live_1805B0C0 = (_d[7] & (0xFFU));
  return Remote_Ctr_1_CANID;
}

uint32_t Pack_Remote_Ctr_1_V1(const Remote_Ctr_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Remote_Ctr_1_DLC; _d[i++] = 0);

  _d[0] |= (_m->RemoteDrivingModeRq & (0x03U)) | ((_m->Door_Cmd & (0x03U)) << 2) | ((_m->Horn_Cmd & (0x03U)) << 4) | ((_m->DaytimeRunningLights_Cmd & (0x03U)) << 6);
  _d[1] |= (_m->Left_Turn_Lamps_Cmd & (0x03U)) | ((_m->Right_Turn_Lamps_Cmd & (0x03U)) << 2) | ((_m->Warning_Lamps_Cmd & (0x03U)) << 4) | ((_m->Positing_Lamps_Cmd & (0x03U)) << 6);
  _d[2] |= (_m->Headlight_Cmd & (0x03U)) | ((_m->Rear_Fog_Lamps_Cmd & (0x03U)) << 2) | ((_m->Left_Front_Fog_Lamps_Cmd & (0x03U)) << 4) | ((_m->Right_Front_Fog_Lamps_Cmd & (0x03U)) << 6);
  _d[3] |= (_m->IntCeilingLamp_Cmd & (0x03U)) | ((_m->IntAtmosphereLamp_Cmd & (0x03U)) << 2);
  _d[4] |= (_m->Remote_System_Connection_Cmd & (0x03U)) | ((_m->Remote_System_Reliability & (0x03U)) << 2) | ((_m->Remote_System_Fault_Level & (0x03U)) << 4) | ((_m->Air_Conditioning_Wind_Speed_Cmd & (0x03U)) << 6);
  _d[5] |= (_m->Air_Conditioning_Mode_Cmd & (0x07U)) | ((_m->Air_Conditioning_Set_Tem_Cmd & (0x1FU)) << 3);
  _d[7] |= (_m->Live_1805B0C0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Remote_Ctr_1_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_BodyCtr_1_V1(BodyCtr_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Door_Cmd = (_d[0] & (0x03U));
  _m->Horn_Cmd = ((_d[0] >> 2) & (0x03U));
  _m->TurnSignalLamp_Cmd = (_d[1] & (0x07U));
  _m->CombinedLamps_Cmd = ((_d[1] >> 4) & (0x0FU));
  _m->Highbeamflash_Cmd = (_d[2] & (0x03U));
  _m->IntCeilingLamp_Cmd = ((_d[2] >> 2) & (0x03U));
  _m->IntAtmosphereLamp_Cmd = ((_d[2] >> 4) & (0x03U));
  _m->Air_Conditioning_Wind_Speed_Cmd = (_d[3] & (0x03U));
  _m->Air_Conditioning_Mode_Cmd = ((_d[3] >> 2) & (0x03U));
  _m->Air_Conditioning_Set_Tem_Cmd = ((_d[4] & (0x1FU))) + 32;
  _m->Live_1805B0D1 = (_d[7] & (0xFFU));
  return BodyCtr_1_CANID;
}

uint32_t Pack_BodyCtr_1_V1(const BodyCtr_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < BodyCtr_1_DLC; _d[i++] = 0);

  _d[0] |= (_m->Door_Cmd & (0x03U)) | ((_m->Horn_Cmd & (0x03U)) << 2);
  _d[1] |= (_m->TurnSignalLamp_Cmd & (0x07U)) | ((_m->CombinedLamps_Cmd & (0x0FU)) << 4);
  _d[2] |= (_m->Highbeamflash_Cmd & (0x03U)) | ((_m->IntCeilingLamp_Cmd & (0x03U)) << 2) | ((_m->IntAtmosphereLamp_Cmd & (0x03U)) << 4);
  _d[3] |= (_m->Air_Conditioning_Wind_Speed_Cmd & (0x03U)) | ((_m->Air_Conditioning_Mode_Cmd & (0x03U)) << 2);
  _d[4] |= (_m->Air_Conditioning_Set_Tem_Cmd & (0x1FU));
  _d[7] |= (_m->Live_1805B0D1 & (0xFFU));
  *_len = 8; *_ide = 1;
  return BodyCtr_1_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_State_1_V1(Vehicle_State_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->DrivingMode_State = (_d[0] & (0x03U));
  _m->Door_State = ((_d[0] >> 2) & (0x01U));
  _m->DaytimeRunningLights_State = ((_d[0] >> 3) & (0x01U));
  _m->Left_Turn_Lamps_State = ((_d[0] >> 4) & (0x01U));
  _m->Right_Turn_Lamps_State = ((_d[0] >> 5) & (0x01U));
  _m->Warning_Lamps_State = ((_d[0] >> 6) & (0x01U));
  _m->Position_Lamps_State = ((_d[0] >> 7) & (0x01U));
  _m->Headlight_State = (_d[1] & (0x03U));
  _m->Rear_Fog_Lamps_State = ((_d[1] >> 2) & (0x01U));
  _m->Left_Front_Fog_Lamps_State = ((_d[1] >> 3) & (0x01U));
  _m->Right_Front_Fog_Lamps_State = ((_d[1] >> 4) & (0x01U));
  _m->IntCeilingLamp_State = ((_d[1] >> 5) & (0x01U));
  _m->IntAtmosphereLamp_State = ((_d[1] >> 6) & (0x01U));
  _m->Door_Button_State = ((_d[1] >> 7) & (0x01U));
  _m->Vehicle_Speed = ((_d[2] & (0xFFU))) - 50;
  _m->Vehicle_SOC_State = (_d[3] & (0xFFU));
  _m->Autopilot_System_State = (_d[4] & (0x03U));
  _m->Vehicle_Charging_State = ((_d[4] >> 2) & (0x03U));
  _m->Vehicle_Charging_Connect_State = ((_d[4] >> 4) & (0x03U));
  _m->Vehicle_Fault_Level = ((_d[4] >> 6) & (0x03U));
  _m->Total_Mileage = ((_d[6] & (0xFFU)) << 8) | (_d[5] & (0xFFU));
  _m->Live_1806A0B0 = (_d[7] & (0xFFU));
  return Vehicle_State_1_CANID;
}

uint32_t Pack_Vehicle_State_1_V1(const Vehicle_State_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_State_1_DLC; _d[i++] = 0);

  _d[0] |= (_m->DrivingMode_State & (0x03U)) | ((_m->Door_State & (0x01U)) << 2) | ((_m->DaytimeRunningLights_State & (0x01U)) << 3) | ((_m->Left_Turn_Lamps_State & (0x01U)) << 4) | ((_m->Right_Turn_Lamps_State & (0x01U)) << 5) | ((_m->Warning_Lamps_State & (0x01U)) << 6) | ((_m->Position_Lamps_State & (0x01U)) << 7);
  _d[1] |= (_m->Headlight_State & (0x03U)) | ((_m->Rear_Fog_Lamps_State & (0x01U)) << 2) | ((_m->Left_Front_Fog_Lamps_State & (0x01U)) << 3) | ((_m->Right_Front_Fog_Lamps_State & (0x01U)) << 4) | ((_m->IntCeilingLamp_State & (0x01U)) << 5) | ((_m->IntAtmosphereLamp_State & (0x01U)) << 6) | ((_m->Door_Button_State & (0x01U)) << 7);
  _d[2] |= (_m->Vehicle_Speed & (0xFFU));
  _d[3] |= (_m->Vehicle_SOC_State & (0xFFU));
  _d[4] |= (_m->Autopilot_System_State & (0x03U)) | ((_m->Vehicle_Charging_State & (0x03U)) << 2) | ((_m->Vehicle_Charging_Connect_State & (0x03U)) << 4) | ((_m->Vehicle_Fault_Level & (0x03U)) << 6);
  _d[5] |= (_m->Total_Mileage & (0xFFU));
  _d[6] |= ((_m->Total_Mileage >> 8) & (0xFFU));
  _d[7] |= (_m->Live_1806A0B0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Vehicle_State_1_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Auto_Ctr_2_V1(Auto_Ctr_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AVAS_Cmd = (_d[0] & (0xFFU));
  _m->Autopilot_Request_Remote_Cmd = (_d[1] & (0x0FU));
  _m->Live_1807B0A0 = (_d[7] & (0xFFU));
  return Auto_Ctr_2_CANID;
}

uint32_t Pack_Auto_Ctr_2_V1(const Auto_Ctr_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Auto_Ctr_2_DLC; _d[i++] = 0);

  _d[0] |= (_m->AVAS_Cmd & (0xFFU));
  _d[1] |= (_m->Autopilot_Request_Remote_Cmd & (0x0FU));
  _d[7] |= (_m->Live_1807B0A0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Auto_Ctr_2_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Remote_Ctr_2_V1(Remote_Ctr_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AVAS_Cmd = (_d[0] & (0xFFU));
  _m->Remote_Takeover_Request = (_d[1] & (0x0FU));
  _m->Live_1807B0C0 = (_d[7] & (0xFFU));
  return Remote_Ctr_2_CANID;
}

uint32_t Pack_Remote_Ctr_2_V1(const Remote_Ctr_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Remote_Ctr_2_DLC; _d[i++] = 0);

  _d[0] |= (_m->AVAS_Cmd & (0xFFU));
  _d[1] |= (_m->Remote_Takeover_Request & (0x0FU));
  _d[7] |= (_m->Live_1807B0C0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Remote_Ctr_2_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_State_5_V1(Vehicle_State_5_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AVAS_State = (_d[0] & (0xFFU));
  _m->DrivingMode_State_2 = (_d[1] & (0x07U));
  _m->Live_1808A0B0 = (_d[7] & (0xFFU));
  return Vehicle_State_5_CANID;
}

uint32_t Pack_Vehicle_State_5_V1(const Vehicle_State_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_State_5_DLC; _d[i++] = 0);

  _d[0] |= (_m->AVAS_State & (0xFFU));
  _d[1] |= (_m->DrivingMode_State_2 & (0x07U));
  _d[7] |= (_m->Live_1808A0B0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Vehicle_State_5_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_State_2_V1(Vehicle_State_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Power_Battery_Voltage = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->Power_Battery_Current = (((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU))) - 25000;
  _m->Accumulated_Charge_Of_Battery = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  _m->Accumulated_Output_Of_Battery = ((_d[7] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
  return Vehicle_State_2_CANID;
}

uint32_t Pack_Vehicle_State_2_V1(const Vehicle_State_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_State_2_DLC; _d[i++] = 0);

  _d[0] |= (_m->Power_Battery_Voltage & (0xFFU));
  _d[1] |= ((_m->Power_Battery_Voltage >> 8) & (0xFFU));
  _d[2] |= (_m->Power_Battery_Current & (0xFFU));
  _d[3] |= ((_m->Power_Battery_Current >> 8) & (0xFFU));
  _d[4] |= (_m->Accumulated_Charge_Of_Battery & (0xFFU));
  _d[5] |= ((_m->Accumulated_Charge_Of_Battery >> 8) & (0xFFU));
  _d[6] |= (_m->Accumulated_Output_Of_Battery & (0xFFU));
  _d[7] |= ((_m->Accumulated_Output_Of_Battery >> 8) & (0xFFU));
  *_len = 8; *_ide = 1;
  return Vehicle_State_2_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_State_3_V1(Vehicle_State_3_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Battery_Temperature_MAX = ((_d[0] & (0xFFU))) - 40;
  _m->Battery_Temperature_MIN = ((_d[1] & (0xFFU))) - 40;
  _m->Cell_Voltagel_MAX = ((_d[3] & (0x0FU)) << 8) | (_d[2] & (0xFFU));
  _m->Cell_Voltage_MIN = ((_d[4] & (0xFFU)) << 4) | ((_d[3] >> 4) & (0x0FU));
  _m->Motor_Temperature = ((_d[5] & (0xFFU))) - 40;
  _m->Motor_Controller_Temperature = ((_d[6] & (0xFFU))) - 40;
  return Vehicle_State_3_CANID;
}

uint32_t Pack_Vehicle_State_3_V1(const Vehicle_State_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_State_3_DLC; _d[i++] = 0);

  _d[0] |= (_m->Battery_Temperature_MAX & (0xFFU));
  _d[1] |= (_m->Battery_Temperature_MIN & (0xFFU));
  _d[2] |= (_m->Cell_Voltagel_MAX & (0xFFU));
  _d[3] |= ((_m->Cell_Voltagel_MAX >> 8) & (0x0FU)) | ((_m->Cell_Voltage_MIN & (0x0FU)) << 4);
  _d[4] |= ((_m->Cell_Voltage_MIN >> 4) & (0xFFU));
  _d[5] |= (_m->Motor_Temperature & (0xFFU));
  _d[6] |= (_m->Motor_Controller_Temperature & (0xFFU));
  *_len = 8; *_ide = 1;
  return Vehicle_State_3_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_State_4_V1(Vehicle_State_4_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Outside_Temperature = ((_d[0] & (0xFFU))) - 60;
  _m->Inside_Temperature = ((_d[1] & (0xFFU))) - 60;
  _m->Air_Conditioning_Mode_State = (_d[2] & (0x0FU));
  _m->Vehicle_Power_State = ((_d[2] >> 4) & (0x03U));
  _m->Pamp_Assist = ((_d[2] >> 6) & (0x01U));
  _m->Regenerative_Barking = ((_d[2] >> 7) & (0x01U));
  _m->Remaining_Mileage = ((_d[4] & (0x0FU)) << 8) | (_d[3] & (0xFFU));
  _m->Manual_Takeover_State = ((_d[4] >> 4) & (0x01U));
  _m->Air_Conditioning_WindSpeed_State = ((_d[4] >> 5) & (0x03U));
  _m->Allow_Remote_Takeover_State = ((_d[4] >> 7) & (0x01U));
  _m->Air_Conditioning_Tem_Set_State = ((_d[5] & (0x1FU))) + 32;
  _m->BrakeLamp = ((_d[5] >> 5) & (0x01U));
  _m->ReversingLamp = ((_d[5] >> 6) & (0x01U));
  _m->ResetButtonState = ((_d[5] >> 7) & (0x01U));
  _m->Battery_Power = (((_d[7] & (0xFFU)) << 8) | (_d[6] & (0xFFU))) - 32500;
  return Vehicle_State_4_CANID;
}

uint32_t Pack_Vehicle_State_4_V1(const Vehicle_State_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_State_4_DLC; _d[i++] = 0);

  _d[0] |= (_m->Outside_Temperature & (0xFFU));
  _d[1] |= (_m->Inside_Temperature & (0xFFU));
  _d[2] |= (_m->Air_Conditioning_Mode_State & (0x0FU)) | ((_m->Vehicle_Power_State & (0x03U)) << 4) | ((_m->Pamp_Assist & (0x01U)) << 6) | ((_m->Regenerative_Barking & (0x01U)) << 7);
  _d[3] |= (_m->Remaining_Mileage & (0xFFU));
  _d[4] |= ((_m->Remaining_Mileage >> 8) & (0x0FU)) | ((_m->Manual_Takeover_State & (0x01U)) << 4) | ((_m->Air_Conditioning_WindSpeed_State & (0x03U)) << 5) | ((_m->Allow_Remote_Takeover_State & (0x01U)) << 7);
  _d[5] |= (_m->Air_Conditioning_Tem_Set_State & (0x1FU)) | ((_m->BrakeLamp & (0x01U)) << 5) | ((_m->ReversingLamp & (0x01U)) << 6) | ((_m->ResetButtonState & (0x01U)) << 7);
  _d[6] |= (_m->Battery_Power & (0xFFU));
  _d[7] |= ((_m->Battery_Power >> 8) & (0xFFU));
  *_len = 8; *_ide = 1;
  return Vehicle_State_4_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_Fault_V1(Vehicle_Fault_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Insulation_FailureLevel = (_d[0] & (0x03U));
  _m->DC_Fault = ((_d[0] >> 2) & (0x01U));
  _m->Low_SOC_Fault = ((_d[0] >> 3) & (0x01U));
  _m->LowVoltage_Cell_Fault = ((_d[0] >> 4) & (0x01U));
  _m->Brake_Booster_Pressure_Fault = ((_d[0] >> 5) & (0x01U));
  _m->VacuumPump_Fault = ((_d[0] >> 6) & (0x01U));
  _m->Vehicle_System_Fault = ((_d[0] >> 7) & (0x01U));
  _m->Battery_Overheating_Fault = (_d[1] & (0x01U));
  _m->Motor_Overheating_Fault = ((_d[1] >> 1) & (0x01U));
  _m->BMS_Fault = ((_d[1] >> 2) & (0x01U));
  _m->Motor_Fault = ((_d[1] >> 3) & (0x01U));
  _m->BMS_Communication_Fault = ((_d[1] >> 4) & (0x01U));
  _m->MCU_Communication_Fault = ((_d[1] >> 5) & (0x01U));
  _m->EPS_Fault = ((_d[1] >> 6) & (0x01U));
  _m->Air_Conditioning_Fault = ((_d[1] >> 7) & (0x01U));
  _m->LeadAcid_Battery_Fault = (_d[2] & (0x01U));
  _m->EPB_Fault_State = ((_d[2] >> 1) & (0x01U));
  _m->EHB_Fault = ((_d[2] >> 2) & (0x01U));
  _m->Key_Lost_Fault = ((_d[2] >> 3) & (0x01U));
  _m->Low_Tire_Pressure_Fault_State = ((_d[2] >> 4) & (0x01U));
  _m->BMS_FailureLevel = (_d[3] & (0x03U));
  _m->MCU_FailureLevel = ((_d[3] >> 2) & (0x03U));
  _m->Unable_AutoMode = ((_d[3] >> 4) & (0x0FU));
  _m->Unable_RemoteMode = (_d[4] & (0x0FU));
  _m->DriveByWire_Controller_Fault = ((_d[4] >> 4) & (0x03U));
  _m->IgnitionWarn_Gear = ((_d[4] >> 6) & (0x01U));
  _m->IgnitionWarn_Pedal = ((_d[4] >> 7) & (0x01U));
  _m->EHB_BrakeFluid_Low_Fault = (_d[5] & (0x01U));
  _m->EHB_BrakeFluid_Leaked_Fault = ((_d[5] >> 1) & (0x01U));
  _m->TempOutsideFault = ((_d[5] >> 2) & (0x01U));
  _m->TempInsideFault = ((_d[5] >> 3) & (0x01U));
  return Vehicle_Fault_CANID;
}

uint32_t Pack_Vehicle_Fault_V1(const Vehicle_Fault_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_Fault_DLC; _d[i++] = 0);

  _d[0] |= (_m->Insulation_FailureLevel & (0x03U)) | ((_m->DC_Fault & (0x01U)) << 2) | ((_m->Low_SOC_Fault & (0x01U)) << 3) | ((_m->LowVoltage_Cell_Fault & (0x01U)) << 4) | ((_m->Brake_Booster_Pressure_Fault & (0x01U)) << 5) | ((_m->VacuumPump_Fault & (0x01U)) << 6) | ((_m->Vehicle_System_Fault & (0x01U)) << 7);
  _d[1] |= (_m->Battery_Overheating_Fault & (0x01U)) | ((_m->Motor_Overheating_Fault & (0x01U)) << 1) | ((_m->BMS_Fault & (0x01U)) << 2) | ((_m->Motor_Fault & (0x01U)) << 3) | ((_m->BMS_Communication_Fault & (0x01U)) << 4) | ((_m->MCU_Communication_Fault & (0x01U)) << 5) | ((_m->EPS_Fault & (0x01U)) << 6) | ((_m->Air_Conditioning_Fault & (0x01U)) << 7);
  _d[2] |= (_m->LeadAcid_Battery_Fault & (0x01U)) | ((_m->EPB_Fault_State & (0x01U)) << 1) | ((_m->EHB_Fault & (0x01U)) << 2) | ((_m->Key_Lost_Fault & (0x01U)) << 3) | ((_m->Low_Tire_Pressure_Fault_State & (0x01U)) << 4);
  _d[3] |= (_m->BMS_FailureLevel & (0x03U)) | ((_m->MCU_FailureLevel & (0x03U)) << 2) | ((_m->Unable_AutoMode & (0x0FU)) << 4);
  _d[4] |= (_m->Unable_RemoteMode & (0x0FU)) | ((_m->DriveByWire_Controller_Fault & (0x03U)) << 4) | ((_m->IgnitionWarn_Gear & (0x01U)) << 6) | ((_m->IgnitionWarn_Pedal & (0x01U)) << 7);
  _d[5] |= (_m->EHB_BrakeFluid_Low_Fault & (0x01U)) | ((_m->EHB_BrakeFluid_Leaked_Fault & (0x01U)) << 1) | ((_m->TempOutsideFault & (0x01U)) << 2) | ((_m->TempInsideFault & (0x01U)) << 3);
  *_len = 8; *_ide = 1;
  return Vehicle_Fault_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_State_6_V1(Vehicle_State_6_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->EPS_StrgWhlAngSpd = ((_d[1] & (0x01U)) << 8) | (_d[0] & (0xFFU));
  _m->Acce_2 = (((_d[2] & (0x0FU)) << 7) | ((_d[1] >> 1) & (0x7FU))) - 1000;
  _m->Vehicle_Speed_2 = (((_d[3] & (0x1FU)) << 4) | ((_d[2] >> 4) & (0x0FU))) - 100;
  _m->Vehicle_Speed_Target = (((_d[4] & (0x3FU)) << 3) | ((_d[3] >> 5) & (0x07U))) - 100;
  _m->Vehicle_Speed_Target_Vd = ((_d[4] >> 6) & (0x01U));
  _m->ResetLampState = ((_d[4] >> 7) & (0x01U));
  _m->ResetButtonState = (_d[5] & (0x01U));
  _m->EPBButtonState = ((_d[5] >> 1) & (0x01U));
  _m->EHB_ParkingReq = ((_d[5] >> 2) & (0x01U));
  _m->EPB_State_2 = ((_d[5] >> 3) & (0x07U));
  _m->Motor_Rotation_Count = (_d[6] & (0xFFU));
  _m->Live_1815A0B0 = (_d[7] & (0xFFU));
  return Vehicle_State_6_CANID;
}

uint32_t Pack_Vehicle_State_6_V1(const Vehicle_State_6_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_State_6_DLC; _d[i++] = 0);

  _d[0] |= (_m->EPS_StrgWhlAngSpd & (0xFFU));
  _d[1] |= ((_m->EPS_StrgWhlAngSpd >> 8) & (0x01U)) | ((_m->Acce_2 & (0x7FU)) << 1);
  _d[2] |= ((_m->Acce_2 >> 7) & (0x0FU)) | ((_m->Vehicle_Speed_2 & (0x0FU)) << 4);
  _d[3] |= ((_m->Vehicle_Speed_2 >> 4) & (0x1FU)) | ((_m->Vehicle_Speed_Target & (0x07U)) << 5);
  _d[4] |= ((_m->Vehicle_Speed_Target >> 3) & (0x3FU)) | ((_m->Vehicle_Speed_Target_Vd & (0x01U)) << 6) | ((_m->ResetLampState & (0x01U)) << 7);
  _d[5] |= (_m->ResetButtonState & (0x01U)) | ((_m->EPBButtonState & (0x01U)) << 1) | ((_m->EHB_ParkingReq & (0x01U)) << 2) | ((_m->EPB_State_2 & (0x07U)) << 3);
  _d[6] |= (_m->Motor_Rotation_Count & (0xFFU));
  _d[7] |= (_m->Live_1815A0B0 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Vehicle_State_6_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_State_7_V1(Vehicle_State_7_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->PowerSequence = (_d[0] & (0x0FU));
  _m->KeyGear_Virtual = ((_d[0] >> 4) & (0x03U));
  _m->EXT_StrgWhlAng = (((_d[2] & (0x7FU)) << 8) | (_d[1] & (0xFFU))) - 9000;
  _m->StrgWhlDriverIntervention = ((_d[2] >> 7) & (0x01U));
  _m->EXT_AcceleratorAperture = (_d[3] & (0x7FU));
  _m->EXT_BrakingAperture = (_d[4] & (0x7FU));
  _m->BrakingDriverIntervention = ((_d[4] >> 7) & (0x01U));
  _m->EHB_Brake_Pressure = (_d[5] & (0xFFU));
  return Vehicle_State_7_CANID;
}

uint32_t Pack_Vehicle_State_7_V1(const Vehicle_State_7_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_State_7_DLC; _d[i++] = 0);

  _d[0] |= (_m->PowerSequence & (0x0FU)) | ((_m->KeyGear_Virtual & (0x03U)) << 4);
  _d[1] |= (_m->EXT_StrgWhlAng & (0xFFU));
  _d[2] |= ((_m->EXT_StrgWhlAng >> 8) & (0x7FU)) | ((_m->StrgWhlDriverIntervention & (0x01U)) << 7);
  _d[3] |= (_m->EXT_AcceleratorAperture & (0x7FU));
  _d[4] |= (_m->EXT_BrakingAperture & (0x7FU)) | ((_m->BrakingDriverIntervention & (0x01U)) << 7);
  _d[5] |= (_m->EHB_Brake_Pressure & (0xFFU));
  *_len = 8; *_ide = 1;
  return Vehicle_State_7_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_State_8_V1(Vehicle_State_8_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->DCDC_Voltage = ((_d[0] & (0xFFU))) + 50;
  _m->DCDC_Current = (_d[1] & (0xFFU));
  _m->StorageBat_Voltage = ((_d[2] & (0xFFU))) + 50;
  _m->DCDC_2_Voltage = ((_d[3] & (0xFFU))) + 50;
  _m->DCDC_2_Current = (_d[4] & (0xFFU));
  _m->StorageBat_2_Voltage = ((_d[5] & (0xFFU))) + 50;
  _m->DCDC_2_Voltage_Vd = (_d[6] & (0x01U));
  _m->DCDC_2_Current_Vd = ((_d[6] >> 1) & (0x01U));
  _m->StorageBat_2_Voltage_Vd = ((_d[6] >> 2) & (0x01U));
  _m->IsAllowAirWork = ((_d[6] >> 3) & (0x01U));
  _m->Horn_State = ((_d[6] >> 4) & (0x01U));
  _m->isBodyCtrlOnline = ((_d[6] >> 5) & (0x01U));
  return Vehicle_State_8_CANID;
}

uint32_t Pack_Vehicle_State_8_V1(const Vehicle_State_8_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_State_8_DLC; _d[i++] = 0);

  _d[0] |= (_m->DCDC_Voltage & (0xFFU));
  _d[1] |= (_m->DCDC_Current & (0xFFU));
  _d[2] |= (_m->StorageBat_Voltage & (0xFFU));
  _d[3] |= (_m->DCDC_2_Voltage & (0xFFU));
  _d[4] |= (_m->DCDC_2_Current & (0xFFU));
  _d[5] |= (_m->StorageBat_2_Voltage & (0xFFU));
  _d[6] |= (_m->DCDC_2_Voltage_Vd & (0x01U)) | ((_m->DCDC_2_Current_Vd & (0x01U)) << 1) | ((_m->StorageBat_2_Voltage_Vd & (0x01U)) << 2) | ((_m->IsAllowAirWork & (0x01U)) << 3) | ((_m->Horn_State & (0x01U)) << 4) | ((_m->isBodyCtrlOnline & (0x01U)) << 5);
  *_len = 8; *_ide = 1;
  return Vehicle_State_8_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Vehicle_VIN_Msg_V1(Vehicle_VIN_Msg_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Device_Type = (_d[0] & (0x3FU));
  _m->ID_Number = ((_d[0] >> 6) & (0x03U));
  _m->VIN_Code_1 = (_d[1] & (0xFFU));
  _m->VIN_Code_2 = (_d[2] & (0xFFU));
  _m->VIN_Code_3 = (_d[3] & (0xFFU));
  _m->VIN_Code_4 = (_d[4] & (0xFFU));
  _m->VIN_Code_5 = (_d[5] & (0xFFU));
  _m->VIN_Code_6 = (_d[6] & (0xFFU));
  _m->VIN_Code_7 = (_d[7] & (0xFFU));
  return Vehicle_VIN_Msg_CANID;
}

uint32_t Pack_Vehicle_VIN_Msg_V1(const Vehicle_VIN_Msg_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Vehicle_VIN_Msg_DLC; _d[i++] = 0);

  _d[0] |= (_m->Device_Type & (0x3FU)) | ((_m->ID_Number & (0x03U)) << 6);
  _d[1] |= (_m->VIN_Code_1 & (0xFFU));
  _d[2] |= (_m->VIN_Code_2 & (0xFFU));
  _d[3] |= (_m->VIN_Code_3 & (0xFFU));
  _d[4] |= (_m->VIN_Code_4 & (0xFFU));
  _d[5] |= (_m->VIN_Code_5 & (0xFFU));
  _d[6] |= (_m->VIN_Code_6 & (0xFFU));
  _d[7] |= (_m->VIN_Code_7 & (0xFFU));
  *_len = 8; *_ide = 1;
  return Vehicle_VIN_Msg_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Version_V1(Version_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Version_Protocol = (_d[0] & (0xFFU));
  _m->Version_FW_Year = ((_d[2] & (0xFFU))) + 2000;
  _m->Version_FW_Months = (_d[3] & (0xFFU));
  _m->Version_FW_Date = (_d[4] & (0xFFU));
  _m->CustomerName_1 = (_d[5] & (0xFFU));
  _m->CustomerName_2 = (_d[6] & (0xFFU));
  _m->isRelease = (_d[7] & (0x01U));
  return Version_CANID;
}

uint32_t Pack_Version_V1(const Version_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Version_DLC; _d[i++] = 0);

  _d[0] |= (_m->Version_Protocol & (0xFFU));
  _d[2] |= (_m->Version_FW_Year & (0xFFU));
  _d[3] |= (_m->Version_FW_Months & (0xFFU));
  _d[4] |= (_m->Version_FW_Date & (0xFFU));
  _d[5] |= (_m->CustomerName_1 & (0xFFU));
  _d[6] |= (_m->CustomerName_2 & (0xFFU));
  _d[7] |= (_m->isRelease & (0x01U));
  *_len = 8; *_ide = 1;
  return Version_CANID;
}

