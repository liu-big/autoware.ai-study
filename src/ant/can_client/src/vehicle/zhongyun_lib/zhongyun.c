/*
 * zhongyun.c
 *
 *  Created on: Mar 27, 2021
 *      Author: chao
 */
#include "zhongyun.h"
#include "proj_can_def.h"
#include "canbase.h"
#include <uclib/osal/osal_mutex.h>
#include <uclib/uc_utils.h>
#define CAN_CMD_ARRAY_SIZE      5

#define FBK_CANID_VCU           0xC1
#define FBK_CANID_BMS           0xC2
#define FBK_CANID_ENA           0xC3
#define FBK_CANID_ERR           0xE1

#define ZYN_ERR_TYPE_BMS        0x01
#define ZYN_ERR_TYPE_MOT        0x02

#define CMD_CANID_GEAR          0xA1
#define CMD_CANID_STEERING      0xA2
#define CMD_CANID_DRIVE         0xA3
#define CMD_CANID_BRAKE         0xA4
#define CMD_CANID_PARKING       0xA5

#define PROTO_PHY2RAW_STEERING(x)   (((x)+28)*1170.275) //(((x)-(-28))/0.0008545)
#define PROTO_PHY2RAW_DRIVE(x)      (((x)+0)*655.7377) //(((x)-(0))/0.001525)
#define PROTO_PHY2RAW_BRAKE(x)      (((x)+0)*655.7377) //(((x)-(0))/0.001525)

#define PROTO_RAW2PHY_STEERING(x)   ((double)(x)*0.0008545 - 28)
#define PROTO_RAW2PHY_DRIVE(x)      ((double)(x)*0.001525 - 0)
#define PROTO_RAW2PHY_BRAKE(x)      ((double)(x)*0.001525 - 0)

#define PROTO_RAW2PHY_SPD(x)                    ((double)(x)*0.1 - 0)
#define PROTO_RAW2PHY_TORQUE(x)                 ((double)(x)*0.19607 - 0)
#define PROTO_RAW2PHY_BRAKE_PEDAL_POS(x)        ((double)(x)*0.39215 - 0)
#define PROTO_RAW2PHY_BAT_VOL(x)                ((double)(x)*0.1 - 0)

#define PROTO_CAN_CIRCLE_TIME_MS        (20)

typedef enum
{
    STATE_IDLE,
    STATE_CHECK,
    STATE_RUN,
    STATE_SLEEP,
    STATE_ERROR
}STATE;

typedef enum
{
    STATE_CHECK_INNER_SEND,
    STATE_CHECK_INNER_CMP,
}STATE_CHECK_INNER;

//0xA1
typedef union _u_proto_cmd_gear
{
    struct _proto_cmd_gear
    {
        uc_uint64_t enable:8;
        uc_uint64_t req:8;
        uc_uint64_t ipc_mode_ctrl:8;
        uc_uint64_t ipc_stop_emergency:8;
        uc_uint64_t resv:32;
    }bit;
    uc_uint64_t all;
}proto_cmd_gear_t;

//0xA2
typedef union _u_proto_cmd_steering
{
    struct _proto_cmd_steering
    {
        uc_uint64_t enable:8;
        uc_uint64_t req:16;
        uc_uint64_t resv:40;
    }bit;
    uc_uint64_t all;
}proto_cmd_steering_t;

//0xA3
typedef union _u_proto_cmd_drive
{
    struct _proto_cmd_drive
    {
        uc_uint64_t enable:8;
        uc_uint64_t req:16;
        uc_uint64_t resv:40;
    }bit;
    uc_uint64_t all;
}proto_cmd_drive_t;

//0xA4
typedef union _u_proto_cmd_brake
{
    struct _proto_cmd_brake
    {
        uc_uint64_t enable:8;
        uc_uint64_t req:16;
        uc_uint64_t resv:40;
    }bit;
    uc_uint64_t all;
}proto_cmd_brake_t;

typedef enum{
    EDGE_BUTTON_NONE,
    EDGE_BUTTON_FALLING,
    EDGE_BUTTON_RASING
}EDGE_BUTTON;

//0xA5
typedef union _u_proto_cmd_parking
{
    struct _proto_cmd_parking
    {
        uc_uint64_t enable:8;
        uc_uint64_t req:8;
        uc_uint64_t resv:48;
    }bit;
    uc_uint64_t all;
}proto_cmd_parking_t;


typedef struct _zyn_can_obj
{
    uc_bool_t  is_started;
    canframe_t cancmd[CAN_CMD_ARRAY_SIZE];
    osal_mutex_t mutex_cancmd;
    struct _zyn_usr_cmd
    {
        double steering;
        double drive;
        double barke;
        int    parking;
        int    gear;
        int    emergency_stop_enable;
        int    ipc_mode_ctl;
    }cmd;

    err_handler cb_error;
    void *      cb_ctx;

    errcode_t   err;

    proto_fbk_vcu_t fbk_vcu;
    proto_fbk_bms_t fbk_bms;
    proto_fbk_enable_t fbk_enable;
    proto_fbk_err_t fbk_err;

    vcu_status_t vcu;
    bms_status_t bms;

    STATE state;
    uc_bool_t autodrive_enable;
    uc_bool_t autodrive_enable_prev;
    uc_uint8_t edge_auto_enable;

    uc_bool_t  force_clear_err_enable;

}zyn_can_obj_t;

static inline uc_uint8_t _can_calculate_bcc_check_sum(const uc_uint8_t *buf, uc_uint16_t wLen)
{
	uc_uint8_t checksum = 0;
	uc_uint16_t i;

	for(i = 0; i < wLen; i++)
        checksum ^= buf[i];

	return checksum;
}

static uc_err_t _zyn_update_cmd_canframe_array_callback(canframe_t *cmd_array , int *size ,void *ctx);
static void     _zyn_canframe_in_callback(const netcan_packet_t *pkt,void *ctx);
static void     _zyn_offline_callback(void *ctx);
static void     _zyn_cmdarray_init(void);

zyn_can_obj_t zyn;

uc_err_t  zyn_can_init()
{
    canbase_init(CAN_CMD_ARRAY_SIZE);
    canbase_register_canframe_in_handler(_zyn_canframe_in_callback,NULL);
    canbase_register_update_cmd_canframe_array_handler(_zyn_update_cmd_canframe_array_callback,NULL);
    canbase_register_offline_handler(_zyn_offline_callback , NULL);
    osal_mutex_init(&zyn.mutex_cancmd);
    _zyn_cmdarray_init();

    return UC_EOK;
}

void zyn_can_deinit()
{
    canbase_deinit();
    osal_mutex_deinit(&zyn.mutex_cancmd);
}
static void _zyn_cmdarray_init(void)
{
    //档位设置为P
    zyn.cancmd[0].type = CAN_FRAME_TYPE_STD;
    zyn.cancmd[0].len = 8;
    zyn.cancmd[0].id = CMD_CANID_GEAR;
    proto_cmd_gear_t *cmd_gear = (proto_cmd_gear_t *)zyn.cancmd[0].data;
    cmd_gear->bit.enable = 1;
    cmd_gear->bit.req = ZYN_GEAR_N;
    cmd_gear->bit.ipc_mode_ctrl = 0;
    cmd_gear->bit.ipc_stop_emergency = 0;

    //转向5
    zyn.cancmd[1].type = CAN_FRAME_TYPE_STD;
    zyn.cancmd[1].len = 8;
    zyn.cancmd[1].id = CMD_CANID_STEERING;
    proto_cmd_steering_t *cmd_steer = (proto_cmd_steering_t *)zyn.cancmd[1].data;
    cmd_steer->bit.enable = 1;
    cmd_steer->bit.req = PROTO_PHY2RAW_STEERING(0);

    //驱动0
    zyn.cancmd[2].type = CAN_FRAME_TYPE_STD;
    zyn.cancmd[2].len = 8;
    zyn.cancmd[2].id = CMD_CANID_DRIVE;
    proto_cmd_drive_t *cmd_drive = (proto_cmd_drive_t *)zyn.cancmd[2].data;
    cmd_drive->bit.enable = 1;
    cmd_drive->bit.req = PROTO_PHY2RAW_DRIVE(0);

    //刹车10
    zyn.cancmd[3].type = CAN_FRAME_TYPE_STD;
    zyn.cancmd[3].len = 8;
    zyn.cancmd[3].id = CMD_CANID_BRAKE;
    proto_cmd_brake_t *cmd_brake = (proto_cmd_brake_t *)zyn.cancmd[3].data;
    cmd_brake->bit.enable = 1;
    cmd_brake->bit.req = PROTO_RAW2PHY_BRAKE(0);

    //拉起电子手刹
    zyn.cancmd[4].type = CAN_FRAME_TYPE_STD;
    zyn.cancmd[4].len = 8;
    zyn.cancmd[4].id = CMD_CANID_PARKING;
    proto_cmd_parking_t *cmd_parking = (proto_cmd_parking_t *)zyn.cancmd[4].data;
    cmd_parking->bit.enable = 1;
    cmd_parking->bit.req = 0;
}

void     zyn_can_register_err_handler(err_handler handle,void *ctx)
{
    zyn.cb_error = handle;
    zyn.cb_ctx = ctx;
}
uc_err_t zyn_can_start()
{
    uc_err_t err = canbase_start(PROTO_CAN_CIRCLE_TIME_MS);
    return err;
}
void     zyn_can_stop()
{
    canbase_stop();
}
uc_bool_t zyn_can_autodrive_enter(int timeoutms)
{
//	if(!canbase_is_ready()) return UC_FALSE;
//
//    if(zyn.state == STATE_RUN) return UC_TRUE;
//
//    zyn.autodrive_enable = 1;
//    if (zyn.autodrive_enable_prev == 0)
//    {
//        zyn.edge_auto_enable = EDGE_BUTTON_RASING;
//        zyn.autodrive_enable_prev = 1;
//    }
//
//    int count = timeoutms / 10;
//
//    if(count < 100) count = 100;
//    while(count--)
//    {
//        osal_thread_delay_ms(10);
//
//        UC_LOG_D("wait for auto drive %d , current state = %d\r\n",count,zyn.state);
//
//        if(zyn.state == STATE_RUN)
//            break;
//    }
//
//    return (count > 0);

	return UC_TRUE;
}
void      zyn_can_autodrive_exit()
{
    zyn.autodrive_enable = 0;
    if(zyn.autodrive_enable_prev)
    {
        zyn.edge_auto_enable = EDGE_BUTTON_FALLING;
        zyn.autodrive_enable_prev = 0;
    }
}

static void _cmd_set_gear(ZYN_GEAR gear,int estop,int ipc_ctl)
{
    //档位设置
    proto_cmd_gear_t *cmd_gear = (proto_cmd_gear_t *)zyn.cancmd[0].data;
    cmd_gear->bit.enable = 1;
    cmd_gear->bit.req = gear;
    cmd_gear->bit.ipc_mode_ctrl = ipc_ctl;
    cmd_gear->bit.ipc_stop_emergency = estop;
}
static void _cmd_set_steering(double value)
{
    //转向
    proto_cmd_steering_t *cmd_steer = (proto_cmd_steering_t *)zyn.cancmd[1].data;
    cmd_steer->bit.enable = 1;
    cmd_steer->bit.req = PROTO_PHY2RAW_STEERING(value);
}
static void _cmd_set_drive(double torque)
{
    //驱动0
    proto_cmd_drive_t *cmd_drive = (proto_cmd_drive_t *)zyn.cancmd[2].data;
    cmd_drive->bit.enable = 1;
    cmd_drive->bit.req = PROTO_PHY2RAW_DRIVE(torque);
}
static void _cmd_set_brake(double value)
{
    //刹车
    proto_cmd_brake_t *cmd_brake = (proto_cmd_brake_t *)zyn.cancmd[3].data;
    cmd_brake->bit.enable = 1;
    cmd_brake->bit.req = PROTO_PHY2RAW_BRAKE(value);
}
static void _cmd_set_parking(int en)
{
    //拉起电子手刹
    proto_cmd_parking_t *cmd_parking = (proto_cmd_parking_t *)zyn.cancmd[4].data;
    cmd_parking->bit.enable = 1;
    cmd_parking->bit.req = en;
}

uc_err_t zyn_can_set_steering(double value)
{
    osal_mutex_lock(&zyn.mutex_cancmd);
    zyn.cmd.steering = value;
    osal_mutex_unlock(&zyn.mutex_cancmd);
    return UC_EOK;
}
uc_err_t zyn_can_set_drive_torque(double value)
{
    osal_mutex_lock(&zyn.mutex_cancmd);
    zyn.cmd.drive = value;
    osal_mutex_unlock(&zyn.mutex_cancmd);
    return UC_EOK;
}
uc_err_t zyn_can_set_brake_torque(double value)
{
    osal_mutex_lock(&zyn.mutex_cancmd);
    zyn.cmd.barke = value;
    osal_mutex_unlock(&zyn.mutex_cancmd);
    return UC_EOK;
}
uc_err_t zyn_can_set_parking_enable(int enable)
{
    osal_mutex_lock(&zyn.mutex_cancmd);
    zyn.cmd.parking = enable;
    osal_mutex_unlock(&zyn.mutex_cancmd);
    return UC_EOK;
}
uc_err_t zyn_can_set_gear(int value)
{
    osal_mutex_lock(&zyn.mutex_cancmd);
    zyn.cmd.gear = value;
    osal_mutex_unlock(&zyn.mutex_cancmd);
    return UC_EOK;
}
uc_err_t zyn_can_set_mode_ctl(int value)
{
    osal_mutex_lock(&zyn.mutex_cancmd);
    zyn.cmd.ipc_mode_ctl = value;
    osal_mutex_unlock(&zyn.mutex_cancmd);
    return UC_EOK;
}
uc_err_t zyn_can_emergency_stop_enable(int en)
{
    osal_mutex_lock(&zyn.mutex_cancmd);
    zyn.cmd.emergency_stop_enable = en;
    osal_mutex_unlock(&zyn.mutex_cancmd);
    return UC_EOK;
}


uc_err_t zyn_can_get_proto_fbk_vcu(proto_fbk_vcu_t *vcu)
{
    vcu->all = zyn.fbk_vcu.all;
    return UC_EOK;
}
uc_err_t zyn_can_get_proto_fbk_bms(proto_fbk_bms_t *bms)
{
    bms->all = zyn.fbk_bms.all;
    return UC_EOK;
}
uc_err_t zyn_can_get_proto_fbk_enable(proto_fbk_enable_t *enable)
{
    enable->all = zyn.fbk_enable.all;
    return UC_EOK;
}
uc_err_t zyn_can_get_proto_fbk_errcode(proto_fbk_err_t *e)
{
    e->all = zyn.fbk_err.all;
    return UC_EOK;
}
uc_err_t zyn_can_get_vcu_status(vcu_status_t *vcu)
{
    *vcu = zyn.vcu;
    return UC_EOK;
}
uc_err_t zyn_can_get_bms_status(bms_status_t *bms)
{
    *bms = zyn.bms;
    return UC_EOK;
}
uc_bool_t zyn_can_is_human_take_over()
{
    return (zyn.fbk_enable.bit.takeover == 0);
}

/**
 * @method _zyn_state_check
 * 自检
 *
 * @return {int}-4：退出自动驾驶 ， -3:系统异常 ， -2: 人工接管 ， -1:timeout ，  0:wait ， 1:selfcheck ok
 */
typedef enum
{
    RET_STATE_CHECK_EXIT = -4,
    RET_STATE_CHECK_ERR = -3,
    RET_STATE_CHECK_HUMAN = -2,
    RET_STATE_CHECK_TIMEOUT = -1,
    RET_STATE_CHECK_WAIT = 0,
    RET_STATE_CHECK_OK = 1
}RET_STATE_CHECK;

static void _zyn_cmdarray_set_check()
{
    _cmd_set_gear(ZYN_GEAR_N,0,1);
    _cmd_set_steering(5);
    _cmd_set_drive(0);
    _cmd_set_brake(10);
    _cmd_set_parking(1);
}
static void _zyn_cmdarray_recover()
{
    _cmd_set_gear(ZYN_GEAR_N,0,1);
    _cmd_set_steering(0);
    _cmd_set_drive(0);
    _cmd_set_brake(0);
    _cmd_set_parking(0); 
}

static RET_STATE_CHECK _zyn_state_check(canframe_t *cmd_array , int *size , void *ctx)
{
    static int waitcount = 0;
    static STATE_CHECK_INNER state_check_inner = STATE_CHECK_INNER_SEND;

    RET_STATE_CHECK ret = RET_STATE_CHECK_WAIT;
//    if(zyn_can_is_human_take_over())
//    {
//        *size = 0;
//        state_check_inner  = STATE_CHECK_INNER_SEND;
//        waitcount = 0;
//        UC_LOG_I("switch to state sleep\r\n");
//        return RET_STATE_CHECK_HUMAN;
//    }
    if(zyn.err.level == ERRCODE_LEVEL_ERROR)
    {
        *size = 0;
        state_check_inner  = STATE_CHECK_INNER_SEND;
        waitcount = 0;
        UC_LOG_I("switch to state error\r\n");
        return RET_STATE_CHECK_ERR;
    }
    if(zyn.autodrive_enable == UC_FALSE)
    {
        *size = 0;
        zyn.state = STATE_IDLE;
        state_check_inner  = STATE_CHECK_INNER_SEND;
        waitcount = 0;
        UC_LOG_I("switch to state idle\r\n");
        return RET_STATE_CHECK_EXIT;
    }
    else
    {
        
        *size = CAN_CMD_ARRAY_SIZE;

        switch (state_check_inner)
        {
        case STATE_CHECK_INNER_SEND:
        {
            _zyn_cmdarray_set_check();

            state_check_inner = STATE_CHECK_INNER_CMP;

            int i = 0;
            for(i = 0;i<CAN_CMD_ARRAY_SIZE;i++)
            {
                cmd_array[i] = zyn.cancmd[i];
            }
        }
        break;

        case STATE_CHECK_INNER_CMP:
        {
            waitcount++;
            UC_LOG_D("wait for check feedback %d\r\n",waitcount);

            if(waitcount > 50)
            {
                if(zyn_can_is_human_take_over())
                {
                    *size = 0;
                    state_check_inner  = STATE_CHECK_INNER_SEND;
                    waitcount = 0;
                    UC_LOG_I("switch to state sleep\r\n");
                    return RET_STATE_CHECK_HUMAN;
                }
            }

            if(waitcount > 20000/PROTO_CAN_CIRCLE_TIME_MS)
            {
                state_check_inner = STATE_CHECK_INNER_SEND;
                ret = RET_STATE_CHECK_TIMEOUT;
                waitcount = 0;
                UC_LOG_W("self check wait timeout , please check VCU ?\r\n");
            }
            else
            {
                int flag_enable = 0;
                int flag_steering = 0;
                int flag_gear = 0;
                int flag_brake = 0;

                //比较使能信号
//                flag_enable =   (zyn.fbk_enable.bit.gear == 1)&&\
//                                (zyn.fbk_enable.bit.steering == 1) && \
//                                (zyn.fbk_enable.bit.drive == 1)&&\
//                                (zyn.fbk_enable.bit.brake == 1)&&\
//                                (zyn.fbk_enable.bit.parking == 1);
                flag_enable =   (zyn.fbk_enable.bit.gear == 1)&&\
                                (zyn.fbk_enable.bit.steering == 1);
                //比较方向盘
                flag_steering = zyn.vcu.steering > 2;
                //比较档位
                flag_gear = (zyn.vcu.gear == ZYN_GEAR_N);
                //比较刹车
//                 flag_brake = (zyn.vcu.brake_pedal_pos > 1);
                flag_brake = 1;

                UC_LOG_I("enable gear %d , steering %d , drive %d , brake %d , parking %d\r\n",\
                		zyn.fbk_enable.bit.gear,\
						zyn.fbk_enable.bit.steering,\
						zyn.fbk_enable.bit.drive,\
						zyn.fbk_enable.bit.brake,\
						zyn.fbk_enable.bit.parking);
                UC_LOG_I("steering %f , gear %d\r\n",zyn.vcu.steering,zyn.vcu.gear);

                if(flag_enable && flag_steering && flag_gear && flag_brake)
                {
                    UC_LOG_I("self check ok , switch to state run\r\n");
                    
                    _zyn_cmdarray_recover();

                    state_check_inner = STATE_CHECK_INNER_SEND;
                    ret = RET_STATE_CHECK_OK;
                    waitcount = 0;

                    int i = 0;
                    for(i = 0;i<CAN_CMD_ARRAY_SIZE;i++)
                    {
                        cmd_array[i] = zyn.cancmd[i];
                    }
                }
            }
        }
        break;
        
        default:
            break;
        }
    }

    return ret;
}
static uc_err_t _zyn_update_cmd_canframe_array_callback(canframe_t *cmd_array , int *size , void *ctx)
{
	UC_LOG_D("zyn state = %d\r\n",zyn.state);

    *size = CAN_CMD_ARRAY_SIZE;
//    _cmd_set_gear(zyn.cmd.gear,zyn.cmd.emergency_stop_enable,zyn.cmd.ipc_mode_ctl);
    _cmd_set_gear(3,zyn.cmd.emergency_stop_enable,zyn.cmd.ipc_mode_ctl);
    _cmd_set_steering(zyn.cmd.steering);
    _cmd_set_drive(zyn.cmd.drive);
    _cmd_set_brake(zyn.cmd.barke);
    _cmd_set_parking(zyn.cmd.parking);

    int i = 0;
    for(i = 0;i<CAN_CMD_ARRAY_SIZE;i++)
    {
        cmd_array[i] = zyn.cancmd[i];
    }
//    switch (zyn.state)
//    {
//    case STATE_IDLE:
//    {
//        *size = 0;
//        if(zyn.autodrive_enable)
//        {
//            zyn.state = STATE_CHECK;
//            UC_LOG_I("switch to state check\r\n");
//        }
//    }
//    break;
//
//    case STATE_CHECK:
//    {
//        RET_STATE_CHECK ret =  _zyn_state_check(cmd_array,size,ctx);
//        if(ret == RET_STATE_CHECK_HUMAN ) zyn.state = STATE_SLEEP;
//        else if(ret == RET_STATE_CHECK_TIMEOUT) zyn.state = STATE_SLEEP;
//        else if(ret == RET_STATE_CHECK_ERR) zyn.state = STATE_ERROR;
//        else if(ret == RET_STATE_CHECK_EXIT) zyn.state = STATE_IDLE;
//        else if(ret == RET_STATE_CHECK_OK) zyn.state = STATE_RUN;
//    }
//    break;
//
//    case STATE_RUN:
//    {
////        if(zyn_can_is_human_take_over())
////        {
////            zyn.state = STATE_SLEEP;
////            UC_LOG_I("switch from RUN to SLEEP\r\n");
////        }
////        if(zyn.err.level == ERRCODE_LEVEL_ERROR)
////        {
////            zyn.state = STATE_ERROR;
////            UC_LOG_I("switch from RUN to ERROR\r\n");
////        }
//        if(!zyn.autodrive_enable)
//        {
//            zyn.state = STATE_IDLE;
//            UC_LOG_I("switch from RUN to IDLE\r\n");
//        }
//
//        if(zyn.state == STATE_RUN)
//        {
////        	static uc_uint64_t t0 = 0;
//            *size = CAN_CMD_ARRAY_SIZE;
//            _cmd_set_gear(zyn.cmd.gear,zyn.cmd.emergency_stop_enable);
//            _cmd_set_steering(zyn.cmd.steering);
//            _cmd_set_drive(zyn.cmd.drive);
//            _cmd_set_brake(zyn.cmd.barke);
//            _cmd_set_parking(zyn.cmd.parking);
//
//            int i = 0;
//            for(i = 0;i<CAN_CMD_ARRAY_SIZE;i++)
//            {
//                cmd_array[i] = zyn.cancmd[i];
//            }
//
////            uc_uint64_t t1 = time_now_us();
////            UC_LOG_D("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& dt =%lld\r\n",t1 - t0);
////            t0 = t1;
//        }
//
//    }
//    break;
//
//    case STATE_SLEEP:
//    {
////    	*size = CAN_CMD_ARRAY_SIZE;
////    	zyn.state = STATE_RUN;
//
//        *size = 0;
//        if(!zyn_can_is_human_take_over())
//        {
//            if(zyn.edge_auto_enable == EDGE_BUTTON_FALLING)
//            {
//                zyn.state = STATE_IDLE;
//                UC_LOG_I("switch from SLEEP to IDLE\r\n");
//            }
//        }
//    }
//    break;
//
//    case STATE_ERROR:
//    {
//        UC_LOG_E("error level = %d , ecode = 0x%08X\r\n",zyn.err.level,zyn.err.code.all);
//
//        if(zyn.force_clear_err_enable)
//        {
//            zyn.state = STATE_IDLE;
//            UC_LOG_I("switch from ERROR to IDLE\r\n");
//        }
//    }
//    break;
//
//    default:
//        break;
//    }
    return UC_EOK;
}
static void     _zyn_canframe_in_callback(const netcan_packet_t *pkt,void *ctx)
{
    UC_LOG_D("pkt canid = 0x%04X \r\n",pkt->can.id);
    hex_dump((const char *)pkt->can.data,8,8);
    switch (pkt->can.id)
    {
    case FBK_CANID_VCU:
        memcpy(&zyn.fbk_vcu.all,pkt->can.data,8);
        zyn.vcu.spd = PROTO_RAW2PHY_SPD(zyn.fbk_vcu.bit.spd);
        zyn.vcu.steering = PROTO_RAW2PHY_STEERING(zyn.fbk_vcu.bit.steering);
        zyn.vcu.gear = zyn.fbk_vcu.bit.gear;
        zyn.vcu.brake_torque = PROTO_RAW2PHY_TORQUE(zyn.fbk_vcu.bit.brake_torque);
        zyn.vcu.brake_pedal_pos = PROTO_RAW2PHY_BRAKE_PEDAL_POS(zyn.fbk_vcu.bit.brake_pedal_pos);
        UC_LOG_D("fbk vcu : spd = %f , steering = %f , gear = %d , brake_torque = %f , brake pos = %f\r\n",
        zyn.vcu.spd,zyn.vcu.steering,zyn.vcu.gear,zyn.vcu.brake_torque,zyn.vcu.brake_pedal_pos);
        break;
    case FBK_CANID_BMS:
        memcpy(&zyn.fbk_bms.all,pkt->can.data,8);
        zyn.bms.soc = zyn.fbk_bms.bit.soc;
        zyn.bms.bat_vol = PROTO_RAW2PHY_BAT_VOL(zyn.fbk_bms.bit.bat_vol);
        zyn.bms.bat_discharge_cur = zyn.fbk_bms.bit.bat_discharge_cur;
        UC_LOG_D("fbk bms : soc = %d , bat vol = %f , bat cur = %d\r\n",zyn.bms.soc,zyn.bms.bat_vol,zyn.bms.bat_discharge_cur);
        break;
    case FBK_CANID_ENA:
        memcpy(&zyn.fbk_enable.all,pkt->can.data,8);
        UC_LOG_D("fbk enable : en = 0x%08X\r\n",zyn.fbk_enable.all);
        break;
    case FBK_CANID_ERR:
        memcpy(&zyn.fbk_err.all,pkt->can.data,8);
        UC_LOG_D("fbk error : err = 0x%0X\r\n",zyn.fbk_err.all);

        if(zyn.fbk_err.bit.type == ZYN_ERR_TYPE_BMS)
        {
            zyn.err.level = ERRCODE_LEVEL_WARNING;
            switch (zyn.fbk_err.bit.code)
            {
            case 1:
            zyn.err.code.bit.bms_element = 1;
                break;
            case 2:
            zyn.err.code.bit.bms_charge = 1;
                break;
            case 3:
            zyn.err.code.bit.bms_vol = 1;
                break;
            case 4:
            zyn.err.code.bit.bms_temper = 1;
                break;
            case 5:
            zyn.err.code.bit.bms_soc_low = 1;
                break;
            default:
                break;
            }
        }
        else if(zyn.fbk_err.bit.type == ZYN_ERR_TYPE_MOT)
        {
            zyn.err.level = ERRCODE_LEVEL_ERROR;
            switch (zyn.fbk_err.bit.code)
            {
            case 1:
            zyn.err.code.bit.mot_steer = 1;
                break;
            case 2:
            zyn.err.code.bit.mot_drive = 1;
                break;
            
            default:
                break;
            } 
        }
        else
        {
            zyn.err.level = ERRCODE_LEVEL_NONE;
            zyn.err.code.bit.bms_element = 0;
            zyn.err.code.bit.bms_charge = 0;
            zyn.err.code.bit.bms_vol = 0;
            zyn.err.code.bit.bms_temper = 0;
            zyn.err.code.bit.bms_soc_low = 0;
            zyn.err.code.bit.mot_steer = 0;
            zyn.err.code.bit.mot_drive = 0;
        }
        if(zyn.fbk_err.bit.brake_err)
        {
            zyn.err.level = ERRCODE_LEVEL_ERROR;
            zyn.err.code.bit.mot_brake = 1;
        }
        else
        {
            zyn.err.code.bit.mot_brake = 0;
        }

        if(zyn.err.level != ERRCODE_LEVEL_NONE)
        {
            if(zyn.cb_error) zyn.cb_error(&zyn.err,zyn.cb_ctx);
        }
        
        break;
    
    default:
        break;
    }
}
static void     _zyn_offline_callback(void *ctx)
{
    zyn.err.code.bit.offline = 1;
    zyn.err.level = ERRCODE_LEVEL_ERROR;
    if(zyn.cb_error) zyn.cb_error(&zyn.err,zyn.cb_ctx);
}
