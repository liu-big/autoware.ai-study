
#include "vehicle/yunle_lib/canbase.hpp"


// this file were genereted by coderdbc.com web service
// any questions - mailto:coderdbc@gmail.com

// --------------------------------------------------------------------------
uint32_t Unpack_CCU_Status__YunleCAN(CCU_Status_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->CCU_ShiftLevel_Sts = (_d[0] & (0x03U));
  _m->CCU_P_Sts = ((_d[0] >> 2) & (0x01U));
  _m->CCU_Ignition_Sts = ((_d[0] >> 3) & (0x03U));
  _m->Steering_Wheel_Direction = ((_d[0] >> 7) & (0x01U));
  _m->CCU_Steering_Wheel_Angle = ((_d[2] & (0x0FU)) << 8) | (_d[1] & (0xFFU));
  _m->CCU_Vehicle_Speed = ((_d[3] & (0x1FU)) << 4) | ((_d[2] >> 4) & (0x0FU));
  _m->CCU_Drive_Mode = ((_d[3] >> 5) & (0x07U));
  _m->CCU_ACC_Level = (_d[4] & (0x03U));
  _m->CCU_Brake_Level = ((_d[4] >> 2) & (0x03U));
  _m->Total_Odometer = ((_d[6] & (0xFFU)) << 12) | ((_d[5] & (0xFFU)) << 4) | ((_d[4] >> 4) & (0x0FU));
  _m->Left_Turn_Light_Sts = (_d[7] & (0x01U));
  _m->Right_Turn_Light_Sts = ((_d[7] >> 1) & (0x01U));
  _m->Hazard_Light_Sts = ((_d[7] >> 2) & (0x01U));
  _m->Position_Light_Sts = ((_d[7] >> 3) & (0x01U));
  _m->LowBeam_Sts = ((_d[7] >> 4) & (0x01U));
  _m->HighBeam_Sts = ((_d[7] >> 5) & (0x01U));
  _m->RearFog_Sts = ((_d[7] >> 6) & (0x01U));
  _m->Horn_Sts = ((_d[7] >> 7) & (0x01U));
  return CCU_Status_CANID;
}

uint32_t Pack_CCU_Status__YunleCAN(const CCU_Status_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < CCU_Status_DLC; _d[i++] = 0);

  _d[0] |= (_m->CCU_ShiftLevel_Sts & (0x03U)) | ((_m->CCU_P_Sts & (0x01U)) << 2) | ((_m->CCU_Ignition_Sts & (0x03U)) << 3) | ((_m->Steering_Wheel_Direction & (0x01U)) << 7);
  _d[1] |= (_m->CCU_Steering_Wheel_Angle & (0xFFU));
  _d[2] |= ((_m->CCU_Steering_Wheel_Angle >> 8) & (0x0FU)) | ((_m->CCU_Vehicle_Speed & (0x0FU)) << 4);
  _d[3] |= ((_m->CCU_Vehicle_Speed >> 4) & (0x1FU)) | ((_m->CCU_Drive_Mode & (0x07U)) << 5);
  _d[4] |= (_m->CCU_ACC_Level & (0x03U)) | ((_m->CCU_Brake_Level & (0x03U)) << 2) | ((_m->Total_Odometer & (0x0FU)) << 4);
  _d[5] |= ((_m->Total_Odometer >> 4) & (0xFFU));
  _d[6] |= ((_m->Total_Odometer >> 12) & (0xFFU));
  _d[7] |= (_m->Left_Turn_Light_Sts & (0x01U)) | ((_m->Right_Turn_Light_Sts & (0x01U)) << 1) | ((_m->Hazard_Light_Sts & (0x01U)) << 2) | ((_m->Position_Light_Sts & (0x01U)) << 3) | ((_m->LowBeam_Sts & (0x01U)) << 4) | ((_m->HighBeam_Sts & (0x01U)) << 5) | ((_m->RearFog_Sts & (0x01U)) << 6) | ((_m->Horn_Sts & (0x01U)) << 7);
  *_len = 8; *_ide = 0;
  return CCU_Status_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_SAS_SM_input_vcu70__YunleCAN(SAS_SM_input_vcu70_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->SAS_Angle = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
  _m->SAS_Angle_Speed = (_d[2] & (0xFFU));
  _m->SAS_Trim_Sts = ((_d[3] >> 5) & (0x01U));
  _m->SAS_Calibration_Sts = ((_d[3] >> 6) & (0x01U));
  _m->SAS_Failure_Sts = ((_d[3] >> 7) & (0x01U));
  _m->SAS_Message_Counter = (_d[4] & (0x0FU));
  _m->SAS_Checksum = ((_d[4] >> 4) & (0x0FU));
  return SAS_SM_input_vcu70_CANID;
}

uint32_t Pack_SAS_SM_input_vcu70__YunleCAN(const SAS_SM_input_vcu70_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < SAS_SM_input_vcu70_DLC; _d[i++] = 0);

  _d[0] |= ((_m->SAS_Angle >> 8) & (0xFFU));
  _d[1] |= (_m->SAS_Angle & (0xFFU));
  _d[2] |= (_m->SAS_Angle_Speed & (0xFFU));
  _d[3] |= ((_m->SAS_Trim_Sts & (0x01U)) << 5) | ((_m->SAS_Calibration_Sts & (0x01U)) << 6) | ((_m->SAS_Failure_Sts & (0x01U)) << 7);
  _d[4] |= (_m->SAS_Message_Counter & (0x0FU)) | ((_m->SAS_Checksum & (0x0FU)) << 4);
  *_len = 8; *_ide = 0;
  return SAS_SM_input_vcu70_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_PadGateWay__YunleCAN(PadGateWay_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->GW_ShiftLevel_Req = (_d[0] & (0x03U));
  _m->GW_ACC_Level = ((_d[0] >> 2) & (0x03U));
  _m->GW_Brake_Level = ((_d[0] >> 4) & (0x03U));
  _m->GW_Drive_Mode_Req = ((_d[0] >> 6) & (0x03U));
  _m->GW_Steering_Wheel_Angle = ((_d[2] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
  _m->GW_Left_Turn_Light_Req = (_d[3] & (0x03U));
  _m->GW_Right_Turn_Light_Req = ((_d[3] >> 2) & (0x03U));
  _m->GW_Hazard_Light_Req = ((_d[3] >> 4) & (0x03U));
  _m->GW_Position_Light_Req = ((_d[3] >> 6) & (0x03U));
  _m->GW_LowBeam_Req = (_d[4] & (0x03U));
  _m->GW_HighBeam_Req = ((_d[4] >> 2) & (0x03U));
  _m->GW_RearFogLight_Req = ((_d[4] >> 4) & (0x03U));
  _m->GW_Horn_Req = ((_d[4] >> 6) & (0x03U));
  _m->GW_Target_Speed = ((_d[6] & (0x03U)) << 8) | (_d[5] & (0xFFU));
  _m->GW_Ebrake = ((_d[6] >> 2) & (0x01U));
  return PadGateWay_CANID;
}

uint32_t Pack_PadGateWay__YunleCAN(const PadGateWay_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < PadGateWay_DLC; _d[i++] = 0);

  _d[0] |= (_m->GW_ShiftLevel_Req & (0x03U)) | ((_m->GW_ACC_Level & (0x03U)) << 2) | ((_m->GW_Brake_Level & (0x03U)) << 4) | ((_m->GW_Drive_Mode_Req & (0x03U)) << 6);
  _d[1] |= (_m->GW_Steering_Wheel_Angle & (0xFFU));
  _d[2] |= ((_m->GW_Steering_Wheel_Angle >> 8) & (0xFFU));
  _d[3] |= (_m->GW_Left_Turn_Light_Req & (0x03U)) | ((_m->GW_Right_Turn_Light_Req & (0x03U)) << 2) | ((_m->GW_Hazard_Light_Req & (0x03U)) << 4) | ((_m->GW_Position_Light_Req & (0x03U)) << 6);
  _d[4] |= (_m->GW_LowBeam_Req & (0x03U)) | ((_m->GW_HighBeam_Req & (0x03U)) << 2) | ((_m->GW_RearFogLight_Req & (0x03U)) << 4) | ((_m->GW_Horn_Req & (0x03U)) << 6);
  _d[5] |= (_m->GW_Target_Speed & (0xFFU));
  _d[6] |= ((_m->GW_Target_Speed >> 8) & (0x03U)) | ((_m->GW_Ebrake & (0x01U)) << 2);
  *_len = 8; *_ide = 0;
  return PadGateWay_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_SCU__YunleCAN(SCU_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->SCU_ShiftLevel_Req = (_d[0] & (0x03U));
  _m->SCU_ACC_Mode = ((_d[0] >> 2) & (0x03U));
  _m->SCU_Brake_Mode = ((_d[0] >> 4) & (0x03U));
  _m->SCU_Drive_Mode_Req = ((_d[0] >> 6) & (0x03U));
  _m->SCU_Steering_Wheel_Angle = ((_d[2] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
  _m->SCU_Target_Speed = ((_d[4] & (0x01U)) << 8) | (_d[3] & (0xFFU));
  _m->SCU_Brk_En = ((_d[4] >> 1) & (0x01U));
  _m->GW_Left_Turn_Light_Req = (_d[5] & (0x03U));
  _m->GW_Right_Turn_Light_Req = ((_d[5] >> 2) & (0x03U));
  _m->GW_Hazard_Light_Req = ((_d[5] >> 4) & (0x03U));
  _m->GW_Position_Light_Req = ((_d[5] >> 6) & (0x03U));
  _m->GW_LowBeam_Req = (_d[6] & (0x03U));
  _m->GW_HighBeam_Req = ((_d[6] >> 2) & (0x03U));
  _m->GW_RearFogLight_Req = ((_d[6] >> 4) & (0x03U));
  _m->GW_Horn_Req = ((_d[6] >> 6) & (0x03U));
  return SCU_CANID;
}

uint32_t Pack_SCU__YunleCAN(const SCU_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < SCU_DLC; _d[i++] = 0);

  _d[0] |= (_m->SCU_ShiftLevel_Req & (0x03U)) | ((_m->SCU_ACC_Mode & (0x03U)) << 2) | ((_m->SCU_Brake_Mode & (0x03U)) << 4) | ((_m->SCU_Drive_Mode_Req & (0x03U)) << 6);
  _d[1] |= (_m->SCU_Steering_Wheel_Angle & (0xFFU));
  _d[2] |= ((_m->SCU_Steering_Wheel_Angle >> 8) & (0xFFU));
  _d[3] |= (_m->SCU_Target_Speed & (0xFFU));
  _d[4] |= ((_m->SCU_Target_Speed >> 8) & (0x01U)) | ((_m->SCU_Brk_En & (0x01U)) << 1);
  _d[5] |= (_m->GW_Left_Turn_Light_Req & (0x03U)) | ((_m->GW_Right_Turn_Light_Req & (0x03U)) << 2) | ((_m->GW_Hazard_Light_Req & (0x03U)) << 4) | ((_m->GW_Position_Light_Req & (0x03U)) << 6);
  _d[6] |= (_m->GW_LowBeam_Req & (0x03U)) | ((_m->GW_HighBeam_Req & (0x03U)) << 2) | ((_m->GW_RearFogLight_Req & (0x03U)) << 4) | ((_m->GW_Horn_Req & (0x03U)) << 6);
  _d[7] |= (_m->SCU_Brake_Coefficient & (0xFFU));
  *_len = 8; *_ide = 0;
  return SCU_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_EPS_Status__YunleCAN(EPS_Status_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->EPS_Angle = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->EPS_System_Status = (_d[2] & (0x03U));
  _m->EPS_Fault_Code = ((_d[4] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
  return EPS_Status_CANID;
}

uint32_t Pack_EPS_Status__YunleCAN(const EPS_Status_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < EPS_Status_DLC; _d[i++] = 0);

  _d[0] |= (_m->EPS_Angle & (0xFFU));
  _d[1] |= ((_m->EPS_Angle >> 8) & (0xFFU));
  _d[2] |= (_m->EPS_System_Status & (0x03U));
  _d[3] |= (_m->EPS_Fault_Code & (0xFFU));
  _d[4] |= ((_m->EPS_Fault_Code >> 8) & (0xFFU));
  *_len = 8; *_ide = 0;
  return EPS_Status_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_CCU_SAS_Info__YunleCAN(CCU_SAS_Info_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->SAS_Angle = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->SAS_Angle_Speed = (_d[2] & (0xFFU));
  _m->SAS_Failure_Sts = (_d[3] & (0x01U));
  _m->SAS_Calibration_Sts = ((_d[3] >> 1) & (0x01U));
  _m->SAS_Trim_Sts = ((_d[3] >> 2) & (0x01U));
  _m->SAS_Message_Counter = (_d[4] & (0x0FU));
  _m->SAS_Checksum = ((_d[4] >> 4) & (0x0FU));
  return CCU_SAS_Info_CANID;
}

uint32_t Pack_CCU_SAS_Info__YunleCAN(const CCU_SAS_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < CCU_SAS_Info_DLC; _d[i++] = 0);

  _d[0] |= (_m->SAS_Angle & (0xFFU));
  _d[1] |= ((_m->SAS_Angle >> 8) & (0xFFU));
  _d[2] |= (_m->SAS_Angle_Speed & (0xFFU));
  _d[3] |= (_m->SAS_Failure_Sts & (0x01U)) | ((_m->SAS_Calibration_Sts & (0x01U)) << 1) | ((_m->SAS_Trim_Sts & (0x01U)) << 2);
  _d[4] |= (_m->SAS_Message_Counter & (0x0FU)) | ((_m->SAS_Checksum & (0x0FU)) << 4);
  *_len = 8; *_ide = 0;
  return CCU_SAS_Info_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_CCU_Req__YunleCAN(CCU_Req_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->CCU_Torque_Req = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->CCU_Torque_Req_Invaild = (_d[2] & (0x01U));
  _m->CCU_Break_Pressure_Req_Val_ATM = ((_d[2] >> 1) & (0x01U));
  _m->CCU_IBC_HP_pressure = ((_d[4] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
  _m->CCU_Break_Pressure_Req = ((_d[6] & (0xFFU)) << 8) | (_d[5] & (0xFFU));
  _m->CCU_Break_Pressure_Req_Val = (_d[7] & (0x01U));
  return CCU_Req_CANID;
}

uint32_t Pack_CCU_Req__YunleCAN(const CCU_Req_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < CCU_Req_DLC; _d[i++] = 0);

  _d[0] |= (_m->CCU_Torque_Req & (0xFFU));
  _d[1] |= ((_m->CCU_Torque_Req >> 8) & (0xFFU));
  _d[2] |= (_m->CCU_Torque_Req_Invaild & (0x01U)) | ((_m->CCU_Break_Pressure_Req_Val_ATM & (0x01U)) << 1);
  _d[3] |= (_m->CCU_IBC_HP_pressure & (0xFFU));
  _d[4] |= ((_m->CCU_IBC_HP_pressure >> 8) & (0xFFU));
  _d[5] |= (_m->CCU_Break_Pressure_Req & (0xFFU));
  _d[6] |= ((_m->CCU_Break_Pressure_Req >> 8) & (0xFFU));
  _d[7] |= (_m->CCU_Break_Pressure_Req_Val & (0x01U));
  *_len = 8; *_ide = 0;
  return CCU_Req_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_LHS1_Torque_Feedback__YunleCAN(LHS1_Torque_Feedback_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Torque_Measured = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->Speed_Measured = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->DC_Link_Current = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  _m->SEQ_Torque = (_d[6] & (0xFFU));
  _m->CS_Torque = (_d[7] & (0xFFU));
  return LHS1_Torque_Feedback_CANID;
}

uint32_t Pack_LHS1_Torque_Feedback__YunleCAN(const LHS1_Torque_Feedback_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < LHS1_Torque_Feedback_DLC; _d[i++] = 0);

  _d[0] |= (_m->Torque_Measured & (0xFFU));
  _d[1] |= ((_m->Torque_Measured >> 8) & (0xFFU));
  _d[2] |= (_m->Speed_Measured & (0xFFU));
  _d[3] |= ((_m->Speed_Measured >> 8) & (0xFFU));
  _d[4] |= (_m->DC_Link_Current & (0xFFU));
  _d[5] |= ((_m->DC_Link_Current >> 8) & (0xFFU));
  _d[6] |= (_m->SEQ_Torque & (0xFFU));
  _d[7] |= (_m->CS_Torque & (0xFFU));
  *_len = 8; *_ide = 0;
  return LHS1_Torque_Feedback_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_RHS1_Torque_Feedback__YunleCAN(RHS1_Torque_Feedback_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Torque_Measured = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->Speed_Measured = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->DC_Link_Current = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  _m->SEQ_Torque = (_d[6] & (0xFFU));
  _m->CS_Torque = (_d[7] & (0xFFU));
  return RHS1_Torque_Feedback_CANID;
}

uint32_t Pack_RHS1_Torque_Feedback__YunleCAN(const RHS1_Torque_Feedback_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < RHS1_Torque_Feedback_DLC; _d[i++] = 0);

  _d[0] |= (_m->Torque_Measured & (0xFFU));
  _d[1] |= ((_m->Torque_Measured >> 8) & (0xFFU));
  _d[2] |= (_m->Speed_Measured & (0xFFU));
  _d[3] |= ((_m->Speed_Measured >> 8) & (0xFFU));
  _d[4] |= (_m->DC_Link_Current & (0xFFU));
  _d[5] |= ((_m->DC_Link_Current >> 8) & (0xFFU));
  _d[6] |= (_m->SEQ_Torque & (0xFFU));
  _d[7] |= (_m->CS_Torque & (0xFFU));
  *_len = 8; *_ide = 0;
  return RHS1_Torque_Feedback_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_TBOX_Sts__YunleCAN(TBOX_Sts_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->TBOX_Ignition_Sts = (_d[0] & (0x03U));
  _m->TBOX_General_Fail_Status = ((_d[0] >> 2) & (0x01U));
  _m->TBOX_WIFIAP_Work_Status = ((_d[0] >> 3) & (0x03U));
  _m->TBOX_BlueTooth_work_Status = ((_d[0] >> 5) & (0x03U));
  _m->TBOX_4G_Work_Status = ((_d[0] >> 7) & (0x01U));
  _m->TBOX_4G_Signal_Level = (_d[1] & (0xFFU));
  _m->TBOX_WIFI_Signal_Level = (_d[2] & (0xFFU));
  _m->TBOX_BlueTooth_Signal_Level = (_d[3] & (0xFFU));
  _m->TBOX_ICAN_Data_Link_Status = (_d[5] & (0x03U));
  _m->TBOX_AirCondition_Status = ((_d[5] >> 2) & (0x01U));
  _m->TBOX_AirCleaner_Status = ((_d[5] >> 3) & (0x01U));
  _m->TBOX_Cloud_Mirror_Status = ((_d[5] >> 4) & (0x01U));
  _m->TBOX_Cameral_Controller_Status = ((_d[5] >> 5) & (0x01U));
  _m->TBOX_ExtLamp_Controller_Status = ((_d[5] >> 6) & (0x01U));
  _m->TBOX_Door_Controller_Status = ((_d[5] >> 7) & (0x01U));
  _m->TBOXI_Checksum = (_d[7] & (0xFFU));
  return TBOX_Sts_CANID;
}

uint32_t Pack_TBOX_Sts__YunleCAN(const TBOX_Sts_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < TBOX_Sts_DLC; _d[i++] = 0);

  _d[0] |= (_m->TBOX_Ignition_Sts & (0x03U)) | ((_m->TBOX_General_Fail_Status & (0x01U)) << 2) | ((_m->TBOX_WIFIAP_Work_Status & (0x03U)) << 3) | ((_m->TBOX_BlueTooth_work_Status & (0x03U)) << 5) | ((_m->TBOX_4G_Work_Status & (0x01U)) << 7);
  _d[1] |= (_m->TBOX_4G_Signal_Level & (0xFFU));
  _d[2] |= (_m->TBOX_WIFI_Signal_Level & (0xFFU));
  _d[3] |= (_m->TBOX_BlueTooth_Signal_Level & (0xFFU));
  _d[5] |= (_m->TBOX_ICAN_Data_Link_Status & (0x03U)) | ((_m->TBOX_AirCondition_Status & (0x01U)) << 2) | ((_m->TBOX_AirCleaner_Status & (0x01U)) << 3) | ((_m->TBOX_Cloud_Mirror_Status & (0x01U)) << 4) | ((_m->TBOX_Cameral_Controller_Status & (0x01U)) << 5) | ((_m->TBOX_ExtLamp_Controller_Status & (0x01U)) << 6) | ((_m->TBOX_Door_Controller_Status & (0x01U)) << 7);
  _d[7] |= (_m->TBOXI_Checksum & (0xFFU));
  *_len = 8; *_ide = 0;
  return TBOX_Sts_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_GPS_Sts_1__YunleCAN(GPS_Sts_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->GPS_Location_Sts = (_d[0] & (0x01U));
  _m->GPS_Northern_or_Southern = ((_d[0] >> 1) & (0x01U));
  _m->GPS_East_or_West = ((_d[0] >> 2) & (0x01U));
  _m->GPS_Speed = ((_d[2] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
  _m->GPS_Direction = ((_d[4] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
  _m->GPS_Num_Of_Satellite = (_d[5] & (0xFFU));
  _m->GPS_Checksum = (_d[7] & (0xFFU));
  return GPS_Sts_1_CANID;
}

uint32_t Pack_GPS_Sts_1__YunleCAN(const GPS_Sts_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < GPS_Sts_1_DLC; _d[i++] = 0);

  _d[0] |= (_m->GPS_Location_Sts & (0x01U)) | ((_m->GPS_Northern_or_Southern & (0x01U)) << 1) | ((_m->GPS_East_or_West & (0x01U)) << 2);
  _d[1] |= (_m->GPS_Speed & (0xFFU));
  _d[2] |= ((_m->GPS_Speed >> 8) & (0xFFU));
  _d[3] |= (_m->GPS_Direction & (0xFFU));
  _d[4] |= ((_m->GPS_Direction >> 8) & (0xFFU));
  _d[5] |= (_m->GPS_Num_Of_Satellite & (0xFFU));
  _d[7] |= (_m->GPS_Checksum & (0xFFU));
  *_len = 8; *_ide = 0;
  return GPS_Sts_1_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_GPS_Sts_2__YunleCAN(GPS_Sts_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->GPS_Longitude = ((_d[3] & (0xFFU)) << 24) | ((_d[2] & (0xFFU)) << 16) | ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->GPS_Latitude = ((_d[7] & (0xFFU)) << 24) | ((_d[6] & (0xFFU)) << 16) | ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return GPS_Sts_2_CANID;
}

uint32_t Pack_GPS_Sts_2__YunleCAN(const GPS_Sts_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < GPS_Sts_2_DLC; _d[i++] = 0);

  _d[0] |= (_m->GPS_Longitude & (0xFFU));
  _d[1] |= ((_m->GPS_Longitude >> 8) & (0xFFU));
  _d[2] |= ((_m->GPS_Longitude >> 16) & (0xFFU));
  _d[3] |= ((_m->GPS_Longitude >> 24) & (0xFFU));
  _d[4] |= (_m->GPS_Latitude & (0xFFU));
  _d[5] |= ((_m->GPS_Latitude >> 8) & (0xFFU));
  _d[6] |= ((_m->GPS_Latitude >> 16) & (0xFFU));
  _d[7] |= ((_m->GPS_Latitude >> 24) & (0xFFU));
  *_len = 8; *_ide = 0;
  return GPS_Sts_2_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Hardware_inpute__YunleCAN(Hardware_inpute_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->hardware_input_ignition = (_d[0] & (0x01U));
  _m->hardware_input_KeyOn = ((_d[0] >> 1) & (0x01U));
  _m->hardware_input_D = ((_d[0] >> 2) & (0x01U));
  _m->hardware_input_N = ((_d[0] >> 3) & (0x01U));
  _m->hardware_input_R = ((_d[0] >> 4) & (0x01U));
  _m->hardware_input_Acc = ((_d[0] >> 5) & (0x01U));
  _m->hardware_input_Dec = ((_d[0] >> 6) & (0x01U));
  _m->hardware_Foot_brake_input = ((_d[0] >> 7) & (0x01U));
  _m->hardware_input_Analog_Weight_V = (_d[1] & (0xFFU));
  _m->HarwareGearCmd = ((_d[2] >> 4) & (0x0FU));
  _m->DrvModFlag = (_d[3] & (0x0FU));
  _m->gearcmd = ((_d[3] >> 4) & (0x0FU));
  _m->GW_Target_Speed = ((_d[5] & (0x03U)) << 8) | (_d[4] & (0xFFU));
  _m->SCU_Target_Speed = ((_d[6] & (0x0FU)) << 6) | ((_d[5] >> 2) & (0x3FU));
  _m->HarwareTargetSpd = ((_d[7] & (0x3FU)) << 4) | ((_d[6] >> 4) & (0x0FU));
  return Hardware_inpute_CANID;
}

uint32_t Pack_Hardware_inpute__YunleCAN(const Hardware_inpute_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Hardware_inpute_DLC; _d[i++] = 0);

  _d[0] |= (_m->hardware_input_ignition & (0x01U)) | ((_m->hardware_input_KeyOn & (0x01U)) << 1) | ((_m->hardware_input_D & (0x01U)) << 2) | ((_m->hardware_input_N & (0x01U)) << 3) | ((_m->hardware_input_R & (0x01U)) << 4) | ((_m->hardware_input_Acc & (0x01U)) << 5) | ((_m->hardware_input_Dec & (0x01U)) << 6) | ((_m->hardware_Foot_brake_input & (0x01U)) << 7);
  _d[1] |= (_m->hardware_input_Analog_Weight_V & (0xFFU));
  _d[2] |= ((_m->HarwareGearCmd & (0x0FU)) << 4);
  _d[3] |= (_m->DrvModFlag & (0x0FU)) | ((_m->gearcmd & (0x0FU)) << 4);
  _d[4] |= (_m->GW_Target_Speed & (0xFFU));
  _d[5] |= ((_m->GW_Target_Speed >> 8) & (0x03U)) | ((_m->SCU_Target_Speed & (0x3FU)) << 2);
  _d[6] |= ((_m->SCU_Target_Speed >> 6) & (0x0FU)) | ((_m->HarwareTargetSpd & (0x0FU)) << 4);
  _d[7] |= ((_m->HarwareTargetSpd >> 4) & (0x3FU));
  *_len = 8; *_ide = 0;
  return Hardware_inpute_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_DebugEn__YunleCAN(DebugEn_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->BMSDebugEn = (_d[0] & (0x01U));
  _m->VelDebugEn = ((_d[0] >> 1) & (0x01U));
  _m->PIDDebugEn = ((_d[0] >> 2) & (0x01U));
  _m->TqDebugEn = ((_d[0] >> 3) & (0x01U));
  _m->ASDebugEn = ((_d[0] >> 4) & (0x01U));
  _m->LightDebugEn = ((_d[0] >> 5) & (0x01U));
  _m->SASDebugEn = ((_d[0] >> 6) & (0x01U));
  _m->ChrgerDebugEn = ((_d[0] >> 7) & (0x01U));
  _m->FaultResetCmd = (_d[1] & (0x01U));
  _m->LTLedCmd = ((_d[1] >> 1) & (0x01U));
  _m->RTLedCmd = ((_d[1] >> 2) & (0x01U));
  _m->HalLedCmd = ((_d[1] >> 3) & (0x01U));
  _m->PosLedCmd = ((_d[1] >> 4) & (0x01U));
  _m->LBLedCmd = ((_d[1] >> 5) & (0x01U));
  _m->HBLedCmd = ((_d[1] >> 6) & (0x01U));
  _m->RFLedCmd = ((_d[1] >> 7) & (0x01U));
  _m->HornCmd = (_d[2] & (0x01U));
  return DebugEn_CANID;
}

uint32_t Pack_DebugEn__YunleCAN(const DebugEn_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < DebugEn_DLC; _d[i++] = 0);

  _d[0] |= (_m->BMSDebugEn & (0x01U)) | ((_m->VelDebugEn & (0x01U)) << 1) | ((_m->PIDDebugEn & (0x01U)) << 2) | ((_m->TqDebugEn & (0x01U)) << 3) | ((_m->ASDebugEn & (0x01U)) << 4) | ((_m->LightDebugEn & (0x01U)) << 5) | ((_m->SASDebugEn & (0x01U)) << 6) | ((_m->ChrgerDebugEn & (0x01U)) << 7);
  _d[1] |= (_m->FaultResetCmd & (0x01U)) | ((_m->LTLedCmd & (0x01U)) << 1) | ((_m->RTLedCmd & (0x01U)) << 2) | ((_m->HalLedCmd & (0x01U)) << 3) | ((_m->PosLedCmd & (0x01U)) << 4) | ((_m->LBLedCmd & (0x01U)) << 5) | ((_m->HBLedCmd & (0x01U)) << 6) | ((_m->RFLedCmd & (0x01U)) << 7);
  _d[2] |= (_m->HornCmd & (0x01U));
  *_len = 8; *_ide = 0;
  return DebugEn_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_ChrgASDebug__YunleCAN(ChrgASDebug_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->PreRelayClsCmd = (_d[0] & (0x01U));
  _m->MainRelayClsCmd = ((_d[0] >> 1) & (0x01U));
  _m->ChrgEnCmd = ((_d[0] >> 2) & (0x01U));
  _m->ChrgVolCmd = ((_d[1] & (0x1FU)) << 5) | ((_d[0] >> 3) & (0x1FU));
  _m->ChrgCurCmd = ((_d[2] & (0x7FU)) << 3) | ((_d[1] >> 5) & (0x07U));
  _m->ASFLHtCmd = ((_d[4] & (0x01U)) << 9) | ((_d[3] & (0xFFU)) << 1) | ((_d[2] >> 7) & (0x01U));
  _m->ASFRHtCmd = ((_d[5] & (0x07U)) << 7) | ((_d[4] >> 1) & (0x7FU));
  _m->ASRLHtCmd = ((_d[6] & (0x1FU)) << 5) | ((_d[5] >> 3) & (0x1FU));
  _m->ASRRHtCmd = ((_d[7] & (0x7FU)) << 3) | ((_d[6] >> 5) & (0x07U));
  return ChrgASDebug_CANID;
}

uint32_t Pack_ChrgASDebug__YunleCAN(const ChrgASDebug_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < ChrgASDebug_DLC; _d[i++] = 0);

  _d[0] |= (_m->PreRelayClsCmd & (0x01U)) | ((_m->MainRelayClsCmd & (0x01U)) << 1) | ((_m->ChrgEnCmd & (0x01U)) << 2) | ((_m->ChrgVolCmd & (0x1FU)) << 3);
  _d[1] |= ((_m->ChrgVolCmd >> 5) & (0x1FU)) | ((_m->ChrgCurCmd & (0x07U)) << 5);
  _d[2] |= ((_m->ChrgCurCmd >> 3) & (0x7FU)) | ((_m->ASFLHtCmd & (0x01U)) << 7);
  _d[3] |= ((_m->ASFLHtCmd >> 1) & (0xFFU));
  _d[4] |= ((_m->ASFLHtCmd >> 9) & (0x01U)) | ((_m->ASFRHtCmd & (0x7FU)) << 1);
  _d[5] |= ((_m->ASFRHtCmd >> 7) & (0x07U)) | ((_m->ASRLHtCmd & (0x1FU)) << 3);
  _d[6] |= ((_m->ASRLHtCmd >> 5) & (0x1FU)) | ((_m->ASRRHtCmd & (0x07U)) << 5);
  _d[7] |= ((_m->ASRRHtCmd >> 3) & (0x7FU));
  *_len = 8; *_ide = 0;
  return ChrgASDebug_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_DrvDebug__YunleCAN(DrvDebug_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->VelCmd = ((_d[1] & (0x03U)) << 8) | (_d[0] & (0xFFU));
  _m->TqCmd = ((_d[2] & (0x0FU)) << 6) | ((_d[1] >> 2) & (0x3FU));
  _m->SASCmd = ((_d[3] & (0xFFU)) << 4) | ((_d[2] >> 4) & (0x0FU));
  _m->VelKp = ((_d[5] & (0x0FU)) << 6) | ((_d[4] >> 2) & (0x3FU));
  _m->VelKi = ((_d[6] & (0x3FU)) << 4) | ((_d[5] >> 4) & (0x0FU));
  _m->VelKd = ((_d[7] & (0xFFU)) << 2) | ((_d[6] >> 6) & (0x03U));
  return DrvDebug_CANID;
}

uint32_t Pack_DrvDebug__YunleCAN(const DrvDebug_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < DrvDebug_DLC; _d[i++] = 0);

  _d[0] |= (_m->VelCmd & (0xFFU));
  _d[1] |= ((_m->VelCmd >> 8) & (0x03U)) | ((_m->TqCmd & (0x3FU)) << 2);
  _d[2] |= ((_m->TqCmd >> 6) & (0x0FU)) | ((_m->SASCmd & (0x0FU)) << 4);
  _d[3] |= ((_m->SASCmd >> 4) & (0xFFU));
  _d[4] |= ((_m->VelKp & (0x3FU)) << 2);
  _d[5] |= ((_m->VelKp >> 6) & (0x0FU)) | ((_m->VelKi & (0x0FU)) << 4);
  _d[6] |= ((_m->VelKi >> 4) & (0x3FU)) | ((_m->VelKd & (0x03U)) << 6);
  _d[7] |= ((_m->VelKd >> 2) & (0xFFU));
  *_len = 8; *_ide = 0;
  return DrvDebug_CANID;
}

uint32_t Unpack_BMS_SOC_BMS_for_C(BMS_SOC_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->BMS_Total_VolBat = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->BMS_current_Vol = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->BMS_current_Cur = (((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU))) - 30000;
  _m->BMS_SOC = ((_d[7] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
  return BMS_SOC_CANID;
}

uint32_t Pack_BMS_SOC_BMS_for_C(const BMS_SOC_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < BMS_SOC_DLC; _d[i++] = 0);

  _d[0] |= (_m->BMS_Total_VolBat & (0xFFU));
  _d[1] |= ((_m->BMS_Total_VolBat >> 8) & (0xFFU));
  _d[2] |= (_m->BMS_current_Vol & (0xFFU));
  _d[3] |= ((_m->BMS_current_Vol >> 8) & (0xFFU));
  _d[4] |= (_m->BMS_current_Cur & (0xFFU));
  _d[5] |= ((_m->BMS_current_Cur >> 8) & (0xFFU));
  _d[6] |= (_m->BMS_SOC & (0xFFU));
  _d[7] |= ((_m->BMS_SOC >> 8) & (0xFFU));
  *_len = 8; *_ide = 1;
  return BMS_SOC_CANID;
}