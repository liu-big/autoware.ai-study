
// this file were genereted by coderdbc.com web service
// any questions - mailto:coderdbc@gmail.com

#pragma once


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// This file must define:
// base monitor struct
// function signature for CRC calculation
// function signature for getting system tick value (100 us step)




// def @ctrl_cmd CAN Message (415552208)
#define ctrl_cmd_IDE (1U)
#define ctrl_cmd_DLC (8U)
#define ctrl_cmd_CANID (0x18C4D2D0U)
// signal: @ctrl_cmd_velocity
#define ctrl_cmd_ctrl_cmd_velocity_CovFactor (0.001)
// conversion value to CAN signal
#define ctrl_cmd_ctrl_cmd_velocity_toS(x) ((uint16_t)((x) / 0.001))
// conversion value from CAN signal
#define ctrl_cmd_ctrl_cmd_velocity_fromS(x) ((x) * 0.001)

// signal: @ctrl_cmd_steering
#define ctrl_cmd_ctrl_cmd_steering_CovFactor (0.01)
// conversion value to CAN signal
#define ctrl_cmd_ctrl_cmd_steering_toS(x) ((int16_t)((x) / 0.01))
// conversion value from CAN signal
#define ctrl_cmd_ctrl_cmd_steering_fromS(x) ((x) * 0.01)

typedef struct
{

  uint8_t ctrl_cmd_gear;                    //      Bits=04.  [ 0     , 0      ]  Unit:''     

  uint16_t ctrl_cmd_velocity;               //      Bits=16.  [ 0     , 0      ]  Unit:''      Factor= 0.001 

  int16_t ctrl_cmd_steering;                //  [-] Bits=16.  [ 0     , 0      ]  Unit:''      Factor= 0.01  

  uint8_t ctrl_cmd_Brake;                   //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t ctrl_cmd_alive_cnt;               //      Bits=04.  [ 0     , 0      ]  Unit:''     

  uint8_t ctrl_cmd_check_bcc;               //      Bits=08.  [ 0     , 0      ]  Unit:''     
} ctrl_cmd_t;

// def @ctrl_fb CAN Message (415552239)
#define ctrl_fb_IDE (1U)
#define ctrl_fb_DLC (8U)
#define ctrl_fb_CANID (0x18C4D2EFU)
// signal: @ctrl_fb_velocity
#define ctrl_fb_ctrl_fb_velocity_CovFactor (0.001)
// conversion value to CAN signal
#define ctrl_fb_ctrl_fb_velocity_toS(x) ((uint16_t)((x) / 0.001))
// conversion value from CAN signal
#define ctrl_fb_ctrl_fb_velocity_fromS(x) ((x) * 0.001)

// signal: @ctrl_fb_steering
#define ctrl_fb_ctrl_fb_steering_CovFactor (0.01)
// conversion value to CAN signal
#define ctrl_fb_ctrl_fb_steering_toS(x) ((int16_t)((x) / 0.01))
// conversion value from CAN signal
#define ctrl_fb_ctrl_fb_steering_fromS(x) ((x) * 0.01)

typedef struct
{

  uint8_t ctrl_fb_gear;                     //      Bits=04.  [ 0     , 0      ]  Unit:''     

  uint16_t ctrl_fb_velocity;                //      Bits=16.  [ 0     , 0      ]  Unit:''      Factor= 0.001 

  int16_t ctrl_fb_steering;                 //  [-] Bits=16.  [ 0     , 0      ]  Unit:''      Factor= 0.01  

  uint8_t ctrl_fb_Brake;                    //      Bits=02.  [ 0     , 0      ]  Unit:''     

  uint8_t ctrl_fb_mode;                     //      Bits=02.  [ 0     , 0      ]  Unit:''     

  uint8_t ctrl_fb_alive_cnt;                //      Bits=04.  [ 0     , 0      ]  Unit:''     

  uint8_t ctrl_fb_check_bcc;                //      Bits=08.  [ 0     , 0      ]  Unit:''     
} ctrl_fb_t;

// def @io_cmd CAN Message (415553488)
#define io_cmd_IDE (1U)
#define io_cmd_DLC (8U)
#define io_cmd_CANID (0x18C4D7D0U)
typedef struct
{

  uint8_t io_cmd_enable;                    //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_cmd_lower_beam_headlamp;       //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_cmd_upper_beam_headlamp;       //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_cmd_turn_lamp;                 //      Bits=02.  [ 0     , 0      ]  Unit:''     

  uint8_t io_cmd_braking_lamp;              //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_cmd_clearance_lamp;            //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_cmd_fog_lamp;                  //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_cmd_speaker;                   //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_cmd_alive_cnt;                 //      Bits=04.  [ 0     , 0      ]  Unit:''     

  uint8_t io_cmd_check_bcc;                 //      Bits=08.  [ 0     , 0      ]  Unit:''     
} io_cmd_t;

// def @lr_wheel_fb CAN Message (415553519)
#define lr_wheel_fb_IDE (1U)
#define lr_wheel_fb_DLC (8U)
#define lr_wheel_fb_CANID (0x18C4D7EFU)
// signal: @lr_wheel_fb_velocity
#define lr_wheel_fb_lr_wheel_fb_velocity_CovFactor (0.001)
// conversion value to CAN signal
#define lr_wheel_fb_lr_wheel_fb_velocity_toS(x) ((int16_t)((x) / 0.001))
// conversion value from CAN signal
#define lr_wheel_fb_lr_wheel_fb_velocity_fromS(x) ((x) * 0.001)

typedef struct
{

  int16_t lr_wheel_fb_velocity;             //  [-] Bits=16.  [ 0     , 0      ]  Unit:''      Factor= 0.001 

  int32_t lr_wheel_fb_pulse;                //  [-] Bits=32.  [ 0     , 0      ]  Unit:''     

  uint8_t lr_wheel_fb_alive_cnt;            //      Bits=04.  [ 0     , 0      ]  Unit:''     

  uint8_t lr_wheel_fb_check_bcc;            //      Bits=08.  [ 0     , 0      ]  Unit:''     
} lr_wheel_fb_t;

// def @rr_wheel_fb CAN Message (415553775)
#define rr_wheel_fb_IDE (1U)
#define rr_wheel_fb_DLC (8U)
#define rr_wheel_fb_CANID (0x18C4D8EFU)
// signal: @rr_wheel_fb_velocity
#define rr_wheel_fb_rr_wheel_fb_velocity_CovFactor (0.001)
// conversion value to CAN signal
#define rr_wheel_fb_rr_wheel_fb_velocity_toS(x) ((int16_t)((x) / 0.001))
// conversion value from CAN signal
#define rr_wheel_fb_rr_wheel_fb_velocity_fromS(x) ((x) * 0.001)

typedef struct
{

  int16_t rr_wheel_fb_velocity;             //  [-] Bits=16.  [ 0     , 0      ]  Unit:''      Factor= 0.001 

  int32_t rr_wheel_fb_pulse;                //  [-] Bits=32.  [ 0     , 0      ]  Unit:''     

  uint8_t rr_wheel_fb_alive_cnt;            //      Bits=04.  [ 0     , 0      ]  Unit:''     

  uint8_t rr_wheel_fb_check_bcc;            //      Bits=08.  [ 0     , 0      ]  Unit:''     
} rr_wheel_fb_t;

// def @io_fb CAN Message (415554287)
#define io_fb_IDE (1U)
#define io_fb_DLC (8U)
#define io_fb_CANID (0x18C4DAEFU)
typedef struct
{

  uint8_t io_fb_enable;                     //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_lower_beam_headlamp;        //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_upper_beam_headlamp;        //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_turn_lamp;                  //      Bits=02.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_braking_lamp;               //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_clearance_lamp;             //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_fog_lamp;                   //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_speaker;                    //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_fl_impact_sensor;           //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_fm_impact_sensor;           //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_fr_impact_sensor;           //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_rl_impact_sensor;           //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_rm_impact_sensor;           //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_rr_impact_sensor;           //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_fl_drop_sensor;             //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_fm_drop_sensor;             //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_fr_drop_sensor;             //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_rl_drop_sensor;             //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_rm_drop_sensor;             //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_rr_drop_sensor;             //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_alive_cnt;                  //      Bits=04.  [ 0     , 0      ]  Unit:''     

  uint8_t io_fb_check_bcc;                  //      Bits=08.  [ 0     , 0      ]  Unit:''     
} io_fb_t;

// def @Drive_fb_MCUEcoder CAN Message (415554799)
#define Drive_fb_MCUEcoder_IDE (1U)
#define Drive_fb_MCUEcoder_DLC (8U)
#define Drive_fb_MCUEcoder_CANID (0x18C4DCEFU)
typedef struct
{

  int32_t Drive_fb_MCUEcoder;                //  [-] Bits=32.  [ 0     , 0      ]  Unit:''     

  uint8_t Drive_fb_MCUEcoder_alive_cnt;      //      Bits=04.  [ 0     , 0      ]  Unit:''     

  uint8_t Drive_fb_MCUEcoder_check_bcc;      //      Bits=08.  [ 0     , 0      ]  Unit:''     
} Drive_fb_MCUEcoder_t;

// def @odo_fb CAN Message (415555311)
#define odo_fb_IDE (1U)
#define odo_fb_DLC (8U)
#define odo_fb_CANID (0x18C4DEEFU)
// signal: @odo_fb_accumulative_mileage
#define odo_fb_odo_fb_accumulative_mileage_CovFactor (0.001)
// conversion value to CAN signal
#define odo_fb_odo_fb_accumulative_mileage_toS(x) ((int32_t)((x) / 0.001))
// conversion value from CAN signal
#define odo_fb_odo_fb_accumulative_mileage_fromS(x) ((x) * 0.001)

// signal: @odo_fb_accumulative_angular
#define odo_fb_odo_fb_accumulative_angular_CovFactor (0.001)
// conversion value to CAN signal
#define odo_fb_odo_fb_accumulative_angular_toS(x) ((int32_t)((x) / 0.001))
// conversion value from CAN signal
#define odo_fb_odo_fb_accumulative_angular_fromS(x) ((x) * 0.001)

typedef struct
{

  int32_t odo_fb_accumulative_mileage;      //  [-] Bits=32.  [ 0     , 0      ]  Unit:''      Factor= 0.001 

  int32_t odo_fb_accumulative_angular;      //  [-] Bits=32.  [ 0     , 0      ]  Unit:''      Factor= 0.001 
} odo_fb_t;

// def @bms_Infor CAN Message (415556079)
#define bms_Infor_IDE (1U)
#define bms_Infor_DLC (8U)
#define bms_Infor_CANID (0x18C4E1EFU)
// signal: @bms_Infor_voltage
#define bms_Infor_bms_Infor_voltage_CovFactor (0.01)
// conversion value to CAN signal
#define bms_Infor_bms_Infor_voltage_toS(x) ((uint16_t)((x) / 0.01))
// conversion value from CAN signal
#define bms_Infor_bms_Infor_voltage_fromS(x) ((x) * 0.01)

// signal: @bms_Infor_current
#define bms_Infor_bms_Infor_current_CovFactor (0.01)
// conversion value to CAN signal
#define bms_Infor_bms_Infor_current_toS(x) ((int16_t)((x) / 0.01))
// conversion value from CAN signal
#define bms_Infor_bms_Infor_current_fromS(x) ((x) * 0.01)

// signal: @bms_Infor_remaining_capacity
#define bms_Infor_bms_Infor_remaining_capacity_CovFactor (0.01)
// conversion value to CAN signal
#define bms_Infor_bms_Infor_remaining_capacity_toS(x) ((uint16_t)((x) / 0.01))
// conversion value from CAN signal
#define bms_Infor_bms_Infor_remaining_capacity_fromS(x) ((x) * 0.01)

typedef struct
{

  uint16_t bms_Infor_voltage;                //      Bits=16.  [ 0     , 0      ]  Unit:''      Factor= 0.01  

  int16_t bms_Infor_current;                 //  [-] Bits=16.  [ 0     , 0      ]  Unit:''      Factor= 0.01  

  uint16_t bms_Infor_remaining_capacity;     //      Bits=16.  [ 0     , 0      ]  Unit:''      Factor= 0.01  

  uint8_t bms_Infor_alive_cnt;               //      Bits=04.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_Infor_check_bcc;               //      Bits=08.  [ 0     , 0      ]  Unit:''     
} bms_Infor_t;

// def @bms_flag_Infor CAN Message (415556335)
#define bms_flag_Infor_IDE (1U)
#define bms_flag_Infor_DLC (8U)
#define bms_flag_Infor_CANID (0x18C4E2EFU)
// signal: @bms_flag_Infor_hight_temperature
#define bms_flag_Infor_bms_flag_Infor_hight_temperature_CovFactor (0.1)
// conversion value to CAN signal
#define bms_flag_Infor_bms_flag_Infor_hight_temperature_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define bms_flag_Infor_bms_flag_Infor_hight_temperature_fromS(x) ((x) * 0.1)

// signal: @bms_flag_Infor_low_temperature
#define bms_flag_Infor_bms_flag_Infor_low_temperature_CovFactor (0.1)
// conversion value to CAN signal
#define bms_flag_Infor_bms_flag_Infor_low_temperature_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define bms_flag_Infor_bms_flag_Infor_low_temperature_fromS(x) ((x) * 0.1)

typedef struct
{

  uint8_t bms_flag_Infor_soc;                    //      Bits=08.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_single_ov;              //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_single_uv;              //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_ov;                     //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_uv;                     //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_charge_ot;              //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_charge_ut;              //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_discharge_ot;           //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_discharge_ut;           //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_charge_oc;              //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_discharge_oc;           //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_short;                  //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_ic_error;               //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_lock_mos;               //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_charge_flag;            //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t reserved;                              //      Bits=06.  [ 0     , 0      ]  Unit:''     

  int16_t bms_flag_Infor_hight_temperature;      //  [-] Bits=12.  [ 0     , 0      ]  Unit:''      Factor= 0.1   

  int16_t bms_flag_Infor_low_temperature;        //  [-] Bits=12.  [ 0     , 0      ]  Unit:''      Factor= 0.1   

  uint8_t bms_flag_Infor_alive_cnt;              //      Bits=04.  [ 0     , 0      ]  Unit:''     

  uint8_t bms_flag_Infor_check_bcc;              //      Bits=08.  [ 0     , 0      ]  Unit:''     
} bms_flag_Infor_t;

uint32_t Unpack_ctrl_cmd_chassis_fr_can_protoc(ctrl_cmd_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_ctrl_cmd_chassis_fr_can_protoc(const ctrl_cmd_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_ctrl_fb_chassis_fr_can_protoc(ctrl_fb_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_ctrl_fb_chassis_fr_can_protoc(const ctrl_fb_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_io_cmd_chassis_fr_can_protoc(io_cmd_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_io_cmd_chassis_fr_can_protoc(const io_cmd_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_lr_wheel_fb_chassis_fr_can_protoc(lr_wheel_fb_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_lr_wheel_fb_chassis_fr_can_protoc(const lr_wheel_fb_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_rr_wheel_fb_chassis_fr_can_protoc(rr_wheel_fb_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_rr_wheel_fb_chassis_fr_can_protoc(const rr_wheel_fb_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_io_fb_chassis_fr_can_protoc(io_fb_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_io_fb_chassis_fr_can_protoc(const io_fb_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_Drive_fb_MCUEcoder_chassis_fr_can_protoc(Drive_fb_MCUEcoder_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_Drive_fb_MCUEcoder_chassis_fr_can_protoc(const Drive_fb_MCUEcoder_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_odo_fb_chassis_fr_can_protoc(odo_fb_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_odo_fb_chassis_fr_can_protoc(const odo_fb_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_bms_Infor_chassis_fr_can_protoc(bms_Infor_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_bms_Infor_chassis_fr_can_protoc(const bms_Infor_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_bms_flag_Infor_chassis_fr_can_protoc(bms_flag_Infor_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_bms_flag_Infor_chassis_fr_can_protoc(const bms_flag_Infor_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);

#ifdef __cplusplus
}
#endif

