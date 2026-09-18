
#include "vehicle/yuhsen_lib/canbase.hpp"


// --------------------------------------------------------------------------
uint32_t Unpack_ctrl_cmd_chassis_fr_can_protoc(ctrl_cmd_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->ctrl_cmd_gear = (_d[0] & (0x0FU));
  _m->ctrl_cmd_velocity = ((_d[2] & (0x0FU)) << 12) | ((_d[1] & (0xFFU)) << 4) | ((_d[0] >> 4) & (0x0FU));
  _m->ctrl_cmd_steering = ((_d[4] & (0x0FU)) << 12) | ((_d[3] & (0xFFU)) << 4) | ((_d[2] >> 4) & (0x0FU));
  _m->ctrl_cmd_Brake = ((_d[4] >> 4) & (0x01U));
  _m->ctrl_cmd_alive_cnt = ((_d[6] >> 4) & (0x0FU));
  _m->ctrl_cmd_check_bcc = (_d[7] & (0xFFU));
  return ctrl_cmd_CANID;
}

uint32_t Pack_ctrl_cmd_chassis_fr_can_protoc(const ctrl_cmd_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < ctrl_cmd_DLC; _d[i++] = 0);

  _d[0] |= (_m->ctrl_cmd_gear & (0x0FU)) | ((_m->ctrl_cmd_velocity & (0x0FU)) << 4);
  _d[1] |= ((_m->ctrl_cmd_velocity >> 4) & (0xFFU));
  _d[2] |= ((_m->ctrl_cmd_velocity >> 12) & (0x0FU)) | ((_m->ctrl_cmd_steering & (0x0FU)) << 4);
  _d[3] |= ((_m->ctrl_cmd_steering >> 4) & (0xFFU));
  _d[4] |= ((_m->ctrl_cmd_steering >> 12) & (0x0FU)) | ((_m->ctrl_cmd_Brake & (0x01U)) << 4);
  _d[6] |= ((_m->ctrl_cmd_alive_cnt & (0x0FU)) << 4);
  _d[7] |= (_m->ctrl_cmd_check_bcc & (0xFFU));
  *_len = 8; *_ide = 1;
  return ctrl_cmd_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_ctrl_fb_chassis_fr_can_protoc(ctrl_fb_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->ctrl_fb_gear = (_d[0] & (0x0FU));
  _m->ctrl_fb_velocity = ((_d[2] & (0x0FU)) << 12) | ((_d[1] & (0xFFU)) << 4) | ((_d[0] >> 4) & (0x0FU));
  _m->ctrl_fb_steering = ((_d[4] & (0x0FU)) << 12) | ((_d[3] & (0xFFU)) << 4) | ((_d[2] >> 4) & (0x0FU));
  _m->ctrl_fb_Brake = ((_d[4] >> 4) & (0x03U));
  _m->ctrl_fb_mode = ((_d[4] >> 6) & (0x03U));
  _m->ctrl_fb_alive_cnt = ((_d[6] >> 4) & (0x0FU));
  _m->ctrl_fb_check_bcc = (_d[7] & (0xFFU));
  return ctrl_fb_CANID;
}

uint32_t Pack_ctrl_fb_chassis_fr_can_protoc(const ctrl_fb_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < ctrl_fb_DLC; _d[i++] = 0);

  _d[0] |= (_m->ctrl_fb_gear & (0x0FU)) | ((_m->ctrl_fb_velocity & (0x0FU)) << 4);
  _d[1] |= ((_m->ctrl_fb_velocity >> 4) & (0xFFU));
  _d[2] |= ((_m->ctrl_fb_velocity >> 12) & (0x0FU)) | ((_m->ctrl_fb_steering & (0x0FU)) << 4);
  _d[3] |= ((_m->ctrl_fb_steering >> 4) & (0xFFU));
  _d[4] |= ((_m->ctrl_fb_steering >> 12) & (0x0FU)) | ((_m->ctrl_fb_Brake & (0x03U)) << 4) | ((_m->ctrl_fb_mode & (0x03U)) << 6);
  _d[6] |= ((_m->ctrl_fb_alive_cnt & (0x0FU)) << 4);
  _d[7] |= (_m->ctrl_fb_check_bcc & (0xFFU));
  *_len = 8; *_ide = 1;
  return ctrl_fb_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_io_cmd_chassis_fr_can_protoc(io_cmd_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->io_cmd_enable = (_d[0] & (0x01U));
  _m->io_cmd_lower_beam_headlamp = (_d[1] & (0x01U));
  _m->io_cmd_upper_beam_headlamp = ((_d[1] >> 1) & (0x01U));
  _m->io_cmd_turn_lamp = ((_d[1] >> 2) & (0x03U));
  _m->io_cmd_braking_lamp = ((_d[1] >> 4) & (0x01U));
  _m->io_cmd_clearance_lamp = ((_d[1] >> 5) & (0x01U));
  _m->io_cmd_fog_lamp = ((_d[1] >> 6) & (0x01U));
  _m->io_cmd_speaker = (_d[2] & (0x01U));
  _m->io_cmd_alive_cnt = ((_d[6] >> 4) & (0x0FU));
  _m->io_cmd_check_bcc = (_d[7] & (0xFFU));
  return io_cmd_CANID;
}

uint32_t Pack_io_cmd_chassis_fr_can_protoc(const io_cmd_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < io_cmd_DLC; _d[i++] = 0);

  _d[0] |= (_m->io_cmd_enable & (0x01U));
  _d[1] |= (_m->io_cmd_lower_beam_headlamp & (0x01U)) | ((_m->io_cmd_upper_beam_headlamp & (0x01U)) << 1) | ((_m->io_cmd_turn_lamp & (0x03U)) << 2) | ((_m->io_cmd_braking_lamp & (0x01U)) << 4) | ((_m->io_cmd_clearance_lamp & (0x01U)) << 5) | ((_m->io_cmd_fog_lamp & (0x01U)) << 6);
  _d[2] |= (_m->io_cmd_speaker & (0x01U));
  _d[6] |= ((_m->io_cmd_alive_cnt & (0x0FU)) << 4);
  _d[7] |= (_m->io_cmd_check_bcc & (0xFFU));
  *_len = 8; *_ide = 1;
  return io_cmd_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_lr_wheel_fb_chassis_fr_can_protoc(lr_wheel_fb_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->lr_wheel_fb_velocity = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->lr_wheel_fb_pulse = ((_d[5] & (0xFFU)) << 24) | ((_d[4] & (0xFFU)) << 16) | ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->lr_wheel_fb_alive_cnt = ((_d[6] >> 4) & (0x0FU));
  _m->lr_wheel_fb_check_bcc = (_d[7] & (0xFFU));
  return lr_wheel_fb_CANID;
}

uint32_t Pack_lr_wheel_fb_chassis_fr_can_protoc(const lr_wheel_fb_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < lr_wheel_fb_DLC; _d[i++] = 0);

  _d[0] |= (_m->lr_wheel_fb_velocity & (0xFFU));
  _d[1] |= ((_m->lr_wheel_fb_velocity >> 8) & (0xFFU));
  _d[2] |= (_m->lr_wheel_fb_pulse & (0xFFU));
  _d[3] |= ((_m->lr_wheel_fb_pulse >> 8) & (0xFFU));
  _d[4] |= ((_m->lr_wheel_fb_pulse >> 16) & (0xFFU));
  _d[5] |= ((_m->lr_wheel_fb_pulse >> 24) & (0xFFU));
  _d[6] |= ((_m->lr_wheel_fb_alive_cnt & (0x0FU)) << 4);
  _d[7] |= (_m->lr_wheel_fb_check_bcc & (0xFFU));
  *_len = 8; *_ide = 1;
  return lr_wheel_fb_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_rr_wheel_fb_chassis_fr_can_protoc(rr_wheel_fb_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->rr_wheel_fb_velocity = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->rr_wheel_fb_pulse = ((_d[5] & (0xFFU)) << 24) | ((_d[4] & (0xFFU)) << 16) | ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->rr_wheel_fb_alive_cnt = ((_d[6] >> 4) & (0x0FU));
  _m->rr_wheel_fb_check_bcc = (_d[7] & (0xFFU));
  return rr_wheel_fb_CANID;
}

uint32_t Pack_rr_wheel_fb_chassis_fr_can_protoc(const rr_wheel_fb_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < rr_wheel_fb_DLC; _d[i++] = 0);

  _d[0] |= (_m->rr_wheel_fb_velocity & (0xFFU));
  _d[1] |= ((_m->rr_wheel_fb_velocity >> 8) & (0xFFU));
  _d[2] |= (_m->rr_wheel_fb_pulse & (0xFFU));
  _d[3] |= ((_m->rr_wheel_fb_pulse >> 8) & (0xFFU));
  _d[4] |= ((_m->rr_wheel_fb_pulse >> 16) & (0xFFU));
  _d[5] |= ((_m->rr_wheel_fb_pulse >> 24) & (0xFFU));
  _d[6] |= ((_m->rr_wheel_fb_alive_cnt & (0x0FU)) << 4);
  _d[7] |= (_m->rr_wheel_fb_check_bcc & (0xFFU));
  *_len = 8; *_ide = 1;
  return rr_wheel_fb_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_io_fb_chassis_fr_can_protoc(io_fb_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->io_fb_enable = (_d[0] & (0x01U));
  _m->io_fb_lower_beam_headlamp = (_d[1] & (0x01U));
  _m->io_fb_upper_beam_headlamp = ((_d[1] >> 1) & (0x01U));
  _m->io_fb_turn_lamp = ((_d[1] >> 2) & (0x03U));
  _m->io_fb_braking_lamp = ((_d[1] >> 4) & (0x01U));
  _m->io_fb_clearance_lamp = ((_d[1] >> 5) & (0x01U));
  _m->io_fb_fog_lamp = ((_d[1] >> 6) & (0x01U));
  _m->io_fb_speaker = (_d[2] & (0x01U));
  _m->io_fb_fl_impact_sensor = (_d[3] & (0x01U));
  _m->io_fb_fm_impact_sensor = ((_d[3] >> 1) & (0x01U));
  _m->io_fb_fr_impact_sensor = ((_d[3] >> 2) & (0x01U));
  _m->io_fb_rl_impact_sensor = ((_d[3] >> 3) & (0x01U));
  _m->io_fb_rm_impact_sensor = ((_d[3] >> 4) & (0x01U));
  _m->io_fb_rr_impact_sensor = ((_d[3] >> 5) & (0x01U));
  _m->io_fb_fl_drop_sensor = (_d[4] & (0x01U));
  _m->io_fb_fm_drop_sensor = ((_d[4] >> 1) & (0x01U));
  _m->io_fb_fr_drop_sensor = ((_d[4] >> 2) & (0x01U));
  _m->io_fb_rl_drop_sensor = ((_d[4] >> 3) & (0x01U));
  _m->io_fb_rm_drop_sensor = ((_d[4] >> 4) & (0x01U));
  _m->io_fb_rr_drop_sensor = ((_d[4] >> 5) & (0x01U));
  _m->io_fb_alive_cnt = ((_d[6] >> 4) & (0x0FU));
  _m->io_fb_check_bcc = (_d[7] & (0xFFU));
  return io_fb_CANID;
}

uint32_t Pack_io_fb_chassis_fr_can_protoc(const io_fb_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < io_fb_DLC; _d[i++] = 0);

  _d[0] |= (_m->io_fb_enable & (0x01U));
  _d[1] |= (_m->io_fb_lower_beam_headlamp & (0x01U)) | ((_m->io_fb_upper_beam_headlamp & (0x01U)) << 1) | ((_m->io_fb_turn_lamp & (0x03U)) << 2) | ((_m->io_fb_braking_lamp & (0x01U)) << 4) | ((_m->io_fb_clearance_lamp & (0x01U)) << 5) | ((_m->io_fb_fog_lamp & (0x01U)) << 6);
  _d[2] |= (_m->io_fb_speaker & (0x01U));
  _d[3] |= (_m->io_fb_fl_impact_sensor & (0x01U)) | ((_m->io_fb_fm_impact_sensor & (0x01U)) << 1) | ((_m->io_fb_fr_impact_sensor & (0x01U)) << 2) | ((_m->io_fb_rl_impact_sensor & (0x01U)) << 3) | ((_m->io_fb_rm_impact_sensor & (0x01U)) << 4) | ((_m->io_fb_rr_impact_sensor & (0x01U)) << 5);
  _d[4] |= (_m->io_fb_fl_drop_sensor & (0x01U)) | ((_m->io_fb_fm_drop_sensor & (0x01U)) << 1) | ((_m->io_fb_fr_drop_sensor & (0x01U)) << 2) | ((_m->io_fb_rl_drop_sensor & (0x01U)) << 3) | ((_m->io_fb_rm_drop_sensor & (0x01U)) << 4) | ((_m->io_fb_rr_drop_sensor & (0x01U)) << 5);
  _d[6] |= ((_m->io_fb_alive_cnt & (0x0FU)) << 4);
  _d[7] |= (_m->io_fb_check_bcc & (0xFFU));
  *_len = 8; *_ide = 1;
  return io_fb_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Drive_fb_MCUEcoder_chassis_fr_can_protoc(Drive_fb_MCUEcoder_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Drive_fb_MCUEcoder = ((_d[3] & (0xFFU)) << 24) | ((_d[2] & (0xFFU)) << 16) | ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->Drive_fb_MCUEcoder_alive_cnt = ((_d[6] >> 4) & (0x0FU));
  _m->Drive_fb_MCUEcoder_check_bcc = (_d[7] & (0xFFU));
  return Drive_fb_MCUEcoder_CANID;
}

uint32_t Pack_Drive_fb_MCUEcoder_chassis_fr_can_protoc(const Drive_fb_MCUEcoder_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Drive_fb_MCUEcoder_DLC; _d[i++] = 0);

  _d[0] |= (_m->Drive_fb_MCUEcoder & (0xFFU));
  _d[1] |= ((_m->Drive_fb_MCUEcoder >> 8) & (0xFFU));
  _d[2] |= ((_m->Drive_fb_MCUEcoder >> 16) & (0xFFU));
  _d[3] |= ((_m->Drive_fb_MCUEcoder >> 24) & (0xFFU));
  _d[6] |= ((_m->Drive_fb_MCUEcoder_alive_cnt & (0x0FU)) << 4);
  _d[7] |= (_m->Drive_fb_MCUEcoder_check_bcc & (0xFFU));
  *_len = 8; *_ide = 1;
  return Drive_fb_MCUEcoder_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_odo_fb_chassis_fr_can_protoc(odo_fb_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->odo_fb_accumulative_mileage = ((_d[3] & (0xFFU)) << 24) | ((_d[2] & (0xFFU)) << 16) | ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->odo_fb_accumulative_angular = ((_d[7] & (0xFFU)) << 24) | ((_d[6] & (0xFFU)) << 16) | ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return odo_fb_CANID;
}

uint32_t Pack_odo_fb_chassis_fr_can_protoc(const odo_fb_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < odo_fb_DLC; _d[i++] = 0);

  _d[0] |= (_m->odo_fb_accumulative_mileage & (0xFFU));
  _d[1] |= ((_m->odo_fb_accumulative_mileage >> 8) & (0xFFU));
  _d[2] |= ((_m->odo_fb_accumulative_mileage >> 16) & (0xFFU));
  _d[3] |= ((_m->odo_fb_accumulative_mileage >> 24) & (0xFFU));
  _d[4] |= (_m->odo_fb_accumulative_angular & (0xFFU));
  _d[5] |= ((_m->odo_fb_accumulative_angular >> 8) & (0xFFU));
  _d[6] |= ((_m->odo_fb_accumulative_angular >> 16) & (0xFFU));
  _d[7] |= ((_m->odo_fb_accumulative_angular >> 24) & (0xFFU));
  *_len = 8; *_ide = 1;
  return odo_fb_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_bms_Infor_chassis_fr_can_protoc(bms_Infor_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->bms_Infor_voltage = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->bms_Infor_current = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->bms_Infor_remaining_capacity = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  _m->bms_Infor_alive_cnt = ((_d[6] >> 4) & (0x0FU));
  _m->bms_Infor_check_bcc = (_d[7] & (0xFFU));
  return bms_Infor_CANID;
}

uint32_t Pack_bms_Infor_chassis_fr_can_protoc(const bms_Infor_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < bms_Infor_DLC; _d[i++] = 0);

  _d[0] |= (_m->bms_Infor_voltage & (0xFFU));
  _d[1] |= ((_m->bms_Infor_voltage >> 8) & (0xFFU));
  _d[2] |= (_m->bms_Infor_current & (0xFFU));
  _d[3] |= ((_m->bms_Infor_current >> 8) & (0xFFU));
  _d[4] |= (_m->bms_Infor_remaining_capacity & (0xFFU));
  _d[5] |= ((_m->bms_Infor_remaining_capacity >> 8) & (0xFFU));
  _d[6] |= ((_m->bms_Infor_alive_cnt & (0x0FU)) << 4);
  _d[7] |= (_m->bms_Infor_check_bcc & (0xFFU));
  *_len = 8; *_ide = 1;
  return bms_Infor_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_bms_flag_Infor_chassis_fr_can_protoc(bms_flag_Infor_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->bms_flag_Infor_soc = (_d[0] & (0xFFU));
  _m->bms_flag_Infor_single_ov = (_d[1] & (0x01U));
  _m->bms_flag_Infor_single_uv = ((_d[1] >> 1) & (0x01U));
  _m->bms_flag_Infor_ov = ((_d[1] >> 2) & (0x01U));
  _m->bms_flag_Infor_uv = ((_d[1] >> 3) & (0x01U));
  _m->bms_flag_Infor_charge_ot = ((_d[1] >> 4) & (0x01U));
  _m->bms_flag_Infor_charge_ut = ((_d[1] >> 5) & (0x01U));
  _m->bms_flag_Infor_discharge_ot = ((_d[1] >> 6) & (0x01U));
  _m->bms_flag_Infor_discharge_ut = ((_d[1] >> 7) & (0x01U));
  _m->bms_flag_Infor_charge_oc = (_d[2] & (0x01U));
  _m->bms_flag_Infor_discharge_oc = ((_d[2] >> 1) & (0x01U));
  _m->bms_flag_Infor_short = ((_d[2] >> 2) & (0x01U));
  _m->bms_flag_Infor_ic_error = ((_d[2] >> 3) & (0x01U));
  _m->bms_flag_Infor_lock_mos = ((_d[2] >> 4) & (0x01U));
  _m->bms_flag_Infor_charge_flag = ((_d[2] >> 5) & (0x01U));
  _m->reserved = ((_d[3] & (0x0FU)) << 2) | ((_d[2] >> 6) & (0x03U));
  _m->bms_flag_Infor_hight_temperature = ((_d[4] & (0xFFU)) << 4) | ((_d[3] >> 4) & (0x0FU));
  _m->bms_flag_Infor_low_temperature = ((_d[6] & (0x0FU)) << 8) | (_d[5] & (0xFFU));
  _m->bms_flag_Infor_alive_cnt = ((_d[6] >> 4) & (0x0FU));
  _m->bms_flag_Infor_check_bcc = (_d[7] & (0xFFU));
  return bms_flag_Infor_CANID;
}

uint32_t Pack_bms_flag_Infor_chassis_fr_can_protoc(const bms_flag_Infor_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < bms_flag_Infor_DLC; _d[i++] = 0);

  _d[0] |= (_m->bms_flag_Infor_soc & (0xFFU));
  _d[1] |= (_m->bms_flag_Infor_single_ov & (0x01U)) | ((_m->bms_flag_Infor_single_uv & (0x01U)) << 1) | ((_m->bms_flag_Infor_ov & (0x01U)) << 2) | ((_m->bms_flag_Infor_uv & (0x01U)) << 3) | ((_m->bms_flag_Infor_charge_ot & (0x01U)) << 4) | ((_m->bms_flag_Infor_charge_ut & (0x01U)) << 5) | ((_m->bms_flag_Infor_discharge_ot & (0x01U)) << 6) | ((_m->bms_flag_Infor_discharge_ut & (0x01U)) << 7);
  _d[2] |= (_m->bms_flag_Infor_charge_oc & (0x01U)) | ((_m->bms_flag_Infor_discharge_oc & (0x01U)) << 1) | ((_m->bms_flag_Infor_short & (0x01U)) << 2) | ((_m->bms_flag_Infor_ic_error & (0x01U)) << 3) | ((_m->bms_flag_Infor_lock_mos & (0x01U)) << 4) | ((_m->bms_flag_Infor_charge_flag & (0x01U)) << 5) | ((_m->reserved & (0x03U)) << 6);
  _d[3] |= ((_m->reserved >> 2) & (0x0FU)) | ((_m->bms_flag_Infor_hight_temperature & (0x0FU)) << 4);
  _d[4] |= ((_m->bms_flag_Infor_hight_temperature >> 4) & (0xFFU));
  _d[5] |= (_m->bms_flag_Infor_low_temperature & (0xFFU));
  _d[6] |= ((_m->bms_flag_Infor_low_temperature >> 8) & (0x0FU)) | ((_m->bms_flag_Infor_alive_cnt & (0x0FU)) << 4);
  _d[7] |= (_m->bms_flag_Infor_check_bcc & (0xFFU));
  *_len = 8; *_ide = 1;
  return bms_flag_Infor_CANID;
}