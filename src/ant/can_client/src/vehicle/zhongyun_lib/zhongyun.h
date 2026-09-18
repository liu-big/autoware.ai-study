/*
 * zhongyun.h
 *
 *  Created on: Mar 27, 2021
 *      Author: chao
 */

#ifndef SRC_ZHONGYUN_H_
#define SRC_ZHONGYUN_H_

#include <stdlib.h>
#include <uclib/uc_def.h>
#include "proj_can_def.h"

BEGIN_C_DECLS

//zyn = zhongyun

//0xC1
typedef union _u_proto_fbk_vcu
{
    struct _proto_fbk_vcu
    {
        uc_uint64_t spd:16;
        uc_uint64_t steering:16;
        uc_uint64_t gear:8;
        uc_uint64_t brake_torque:8;
        uc_uint64_t brake_pedal_pos:8;
        uc_uint64_t checksum:8;
    }bit;
    uc_uint64_t all;
}proto_fbk_vcu_t;

//0xC2
typedef union _u_proto_fbk_bms
{
    struct _proto_fbk_bms
    {
        uc_uint64_t soc:8;
        uc_uint64_t bat_vol:16;
        uc_uint64_t bat_discharge_cur:8;
        uc_uint64_t resv:24;
        uc_uint64_t checksum:8;
    }bit;
    uc_uint64_t all;
}proto_fbk_bms_t;

//0xC3
typedef union _u_proto_fbk_enable
{
    struct _proto_fbk_enable
    {
        uc_uint64_t gear:8;
        uc_uint64_t steering:8;
        uc_uint64_t drive:8;
        uc_uint64_t brake:8;
        uc_uint64_t parking:8;
        uc_uint64_t takeover:8;
        uc_uint64_t resv:8;
        uc_uint64_t checksum:8;
    }bit;
    uc_uint64_t all;
}proto_fbk_enable_t;

//0xE1
typedef union _u_proto_fbk_err
{
    struct _proto_fbk_err
    {
        uc_uint64_t type:8;
        uc_uint64_t code:8;
        uc_uint64_t brake_err:8;
        uc_uint64_t resv:32;
        uc_uint64_t checksum:8;
    }bit;
    uc_uint64_t all;
}proto_fbk_err_t;



typedef struct _vcu_status
{
    double spd;//km/h
    double steering;//degree
    int gear;//1:P 2:N 3:D 4:R
    double brake_torque;//当前制动力矩n.m
    double brake_pedal_pos;//当前制动踏板实际开度位置%
}vcu_status_t;


typedef struct _bms_status
{
    int soc;//电量
    double bat_vol;//电压
    int bat_discharge_cur;//电池放电电流
}bms_status_t;

typedef enum
{
    ZYN_GEAR_P = 1,
    ZYN_GEAR_N = 2,
    ZYN_GEAR_D = 3,
    ZYN_GEAR_R = 4
}ZYN_GEAR;

uc_err_t zyn_can_init();
void     zyn_can_deinit();
void     zyn_can_register_err_handler(err_handler handle,void *ctx);
uc_err_t zyn_can_start();
void     zyn_can_stop();

/**
 * 进入自动驾驶
 * */
uc_bool_t zyn_can_autodrive_enter(int timeoutms);
/**
 * 退出自动驾驶
 * */
void      zyn_can_autodrive_exit();

/**
 * 设置车辆转向、驱动、刹车、电子手刹、档位、急停
 * */
uc_err_t zyn_can_set_steering(double value);
uc_err_t zyn_can_set_drive_torque(double value);
uc_err_t zyn_can_set_brake_torque(double value);
uc_err_t zyn_can_set_parking_enable(int enable);
uc_err_t zyn_can_set_gear(int value);
uc_err_t zyn_can_set_mode_ctl(int value);
uc_err_t zyn_can_emergency_stop_enable(int en);

/**
 * 获取原如CAN总线状态反馈数据 vcu bms enable errcode
 * */
uc_err_t zyn_can_get_proto_fbk_vcu(proto_fbk_vcu_t *vcu);
uc_err_t zyn_can_get_proto_fbk_bms(proto_fbk_bms_t *bms);
uc_err_t zyn_can_get_proto_fbk_enable(proto_fbk_enable_t *enable);
uc_err_t zyn_can_get_proto_fbk_errcode(proto_fbk_err_t *e);

/**
 * 获取车辆实际物理意义信息 vcu bms 
 * */
uc_err_t zyn_can_get_vcu_status(vcu_status_t *vcu);
uc_err_t zyn_can_get_bms_status(bms_status_t *bms);

/**
 * @method zyn_can_is_human_take_over
 * 是否人工接管
 *
 * @return {uc_bool_t} 返回UC_TRUE表示人工接管，否则表示自动驾驶中。
 */
uc_bool_t zyn_can_is_human_take_over();

END_C_DECLS

#endif /* SRC_ZHONGYUN_H_ */
