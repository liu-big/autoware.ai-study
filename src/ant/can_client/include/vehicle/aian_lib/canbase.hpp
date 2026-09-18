#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef uint32_t ubitext_t;
typedef int32_t bitext_t;

// DBC file version
#define VER_TESTDB_MAJ (0U)
#define VER_TESTDB_MIN (0U)

// include current dbc-driver compilation config
// #include <testdb-config.h>

#ifdef TESTDB_USE_DIAG_MONITORS
// This file must define:
// base monitor struct
#include <canmonitorutil.h>

#endif // TESTDB_USE_DIAG_MONITORS


// def @IDS_DCU_MSG100 CAN Message (256  0x100)
#define IDS_DCU_MSG100_IDE (0U)
#define IDS_DCU_MSG100_DLC (8U)
#define IDS_DCU_MSG100_CANID (0x100)
#define IDS_DCU_MSG100_CYC (20U)

// Value tables for @IDS_DCU_AutoCtrlEna signal

#ifndef IDS_DCU_AutoCtrlEna_IDS_DCU_MSG100_
#define IDS_DCU_AutoCtrlEna_IDS_DCU_MSG100_ (0)
#endif

#ifndef IDS_DCU_AutoCtrlEna_IDS_DCU_MSG100_
#define IDS_DCU_AutoCtrlEna_IDS_DCU_MSG100_ (1)
#endif


// Value tables for @IDS_ACU_TakeOverMode signal

#ifndef IDS_ACU_TakeOverMode_IDS_DCU_MSG100_
#define IDS_ACU_TakeOverMode_IDS_DCU_MSG100_ (0)
#endif

#ifndef IDS_ACU_TakeOverMode_IDS_DCU_MSG100_
#define IDS_ACU_TakeOverMode_IDS_DCU_MSG100_ (1)
#endif


// Value tables for @IDS_ACU_VehManulToAutoMode signal

#ifndef IDS_ACU_VehManulToAutoMode_IDS_DCU_MSG100_
#define IDS_ACU_VehManulToAutoMode_IDS_DCU_MSG100_ (0)
#endif

#ifndef IDS_ACU_VehManulToAutoMode_IDS_DCU_MSG100_
#define IDS_ACU_VehManulToAutoMode_IDS_DCU_MSG100_ (1)
#endif


typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  //  0 : "����"
  //  1 : "ʹ��"
  uint8_t IDS_DCU_AutoCtrlEna : 1;           //      Bits= 1 Unit:'-'

  //  0 : "ͳһ�ӹ�"
  //  1 : "�ֿ��ӹ�"
  uint8_t IDS_ACU_TakeOverMode : 1;          //      Bits= 1 Unit:'-'

  //  0 : "���ٲ��ܽ��߿�"
  //  1 : "ʹ�ܸ��ٽ��߿�"
  uint8_t IDS_ACU_VehManulToAutoMode : 1;    //      Bits= 1 Unit:'-'

  // ���ļ�����
  uint8_t IDS_DCU_ModeCtrl_CTROLL : 4;       //      Bits= 4 Unit:'-'

  // ����У���
  uint8_t IDS_DCU_ModeCtrl_CHECKSUM;         //      Bits= 8 Unit:'-'

#else

  //  0 : "����"
  //  1 : "ʹ��"
  uint8_t IDS_DCU_AutoCtrlEna;               //      Bits= 1 Unit:'-'

  //  0 : "ͳһ�ӹ�"
  //  1 : "�ֿ��ӹ�"
  uint8_t IDS_ACU_TakeOverMode;              //      Bits= 1 Unit:'-'

  //  0 : "���ٲ��ܽ��߿�"
  //  1 : "ʹ�ܸ��ٽ��߿�"
  uint8_t IDS_ACU_VehManulToAutoMode;        //      Bits= 1 Unit:'-'

  // ���ļ�����
  uint8_t IDS_DCU_ModeCtrl_CTROLL;           //      Bits= 4 Unit:'-'

  // ����У���
  uint8_t IDS_DCU_ModeCtrl_CHECKSUM;         //      Bits= 8 Unit:'-'

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} IDS_DCU_MSG100_t;

// def @IDS_DCU_MSG101 CAN Message (257  0x101)
#define IDS_DCU_MSG101_IDE (0U)
#define IDS_DCU_MSG101_DLC (8U)
#define IDS_DCU_MSG101_CANID (0x101)
#define IDS_DCU_MSG101_CYC (20U)
// signal: @IDS_DCU_AccPedCmd_ro
#define TESTDB_IDS_DCU_AccPedCmd_ro_CovFactor (0.1)
#define TESTDB_IDS_DCU_AccPedCmd_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_IDS_DCU_AccPedCmd_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

// Value tables for @IDS_DCU_AccCtrMode signal

#ifndef IDS_DCU_AccCtrMode_IDS_DCU_MSG101_
#define IDS_DCU_AccCtrMode_IDS_DCU_MSG101_ (0)
#endif

#ifndef IDS_DCU_AccCtrMode_IDS_DCU_MSG101_
#define IDS_DCU_AccCtrMode_IDS_DCU_MSG101_ (1)
#endif


// Value tables for @IDS_DCU_AccOverRideDisable signal

#ifndef IDS_DCU_AccOverRideDisable_IDS_DCU_MSG101_
#define IDS_DCU_AccOverRideDisable_IDS_DCU_MSG101_ (0)
#endif

#ifndef IDS_DCU_AccOverRideDisable_IDS_DCU_MSG101_
#define IDS_DCU_AccOverRideDisable_IDS_DCU_MSG101_ (1)
#endif


// Value tables for @IDS_DCU_AccCtrlEna signal

#ifndef IDS_DCU_AccCtrlEna_IDS_DCU_MSG101_
#define IDS_DCU_AccCtrlEna_IDS_DCU_MSG101_ (0)
#endif

#ifndef IDS_DCU_AccCtrlEna_IDS_DCU_MSG101_
#define IDS_DCU_AccCtrlEna_IDS_DCU_MSG101_ (1)
#endif

// signal: @IDS_DCU_AccPedInv_ro
#define TESTDB_IDS_DCU_AccPedInv_ro_CovFactor (0.1)
#define TESTDB_IDS_DCU_AccPedInv_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_IDS_DCU_AccPedInv_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @IDS_DCU_WheelTorqCmd_ro
#define TESTDB_IDS_DCU_WheelTorqCmd_ro_CovFactor (1)
#define TESTDB_IDS_DCU_WheelTorqCmd_ro_toS(x) ( (uint16_t) ((x) - (-5000)) )
#define TESTDB_IDS_DCU_WheelTorqCmd_ro_fromS(x) ( ((x) + (-5000)) )
// signal: @IDS_DCU_WheelTorqInv_ro
#define TESTDB_IDS_DCU_WheelTorqInv_ro_CovFactor (1)
#define TESTDB_IDS_DCU_WheelTorqInv_ro_toS(x) ( (uint16_t) ((x) - (-5000)) )
#define TESTDB_IDS_DCU_WheelTorqInv_ro_fromS(x) ( ((x) + (-5000)) )

typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  uint16_t IDS_DCU_AccPedCmd_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_AccPedCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "���ſ���"
  //  1 : "Ť�ؿ���"
  uint8_t IDS_DCU_AccCtrMode : 1;            //      Bits= 1 Unit:'-'

  //  0 : "�������Žӹ��˳��Զ�ģʽ"
  //  1 : "ʹ�����Žӹܺ��˳��Զ�ģʽ"
  uint8_t IDS_DCU_AccOverRideDisable : 1;    //      Bits= 1 Unit:'-'

  //  0 : "����"
  //  1 : "ʹ��"
  uint8_t IDS_DCU_AccCtrlEna : 1;            //      Bits= 1 Unit:'-'

  uint16_t IDS_DCU_AccPedInv_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_AccPedInv_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t IDS_DCU_WheelTorqCmd_ro;          //      Bits=14 Offset= -5000              Unit:'Nm'

#ifdef TESTDB_USE_SIGFLOAT
  int16_t IDS_DCU_WheelTorqCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t IDS_DCU_WheelTorqInv_ro;          //      Bits=14 Offset= -5000              Unit:'Nm'

#ifdef TESTDB_USE_SIGFLOAT
  int16_t IDS_DCU_WheelTorqInv_phys;
#endif // TESTDB_USE_SIGFLOAT

#else

  uint16_t IDS_DCU_AccPedCmd_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_AccPedCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "���ſ���"
  //  1 : "Ť�ؿ���"
  uint8_t IDS_DCU_AccCtrMode;                //      Bits= 1 Unit:'-'

  //  0 : "�������Žӹ��˳��Զ�ģʽ"
  //  1 : "ʹ�����Žӹܺ��˳��Զ�ģʽ"
  uint8_t IDS_DCU_AccOverRideDisable;        //      Bits= 1 Unit:'-'

  //  0 : "����"
  //  1 : "ʹ��"
  uint8_t IDS_DCU_AccCtrlEna;                //      Bits= 1 Unit:'-'

  uint16_t IDS_DCU_AccPedInv_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_AccPedInv_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t IDS_DCU_WheelTorqCmd_ro;          //      Bits=14 Offset= -5000              Unit:'Nm'

#ifdef TESTDB_USE_SIGFLOAT
  int16_t IDS_DCU_WheelTorqCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t IDS_DCU_WheelTorqInv_ro;          //      Bits=14 Offset= -5000              Unit:'Nm'

#ifdef TESTDB_USE_SIGFLOAT
  int16_t IDS_DCU_WheelTorqInv_phys;
#endif // TESTDB_USE_SIGFLOAT

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} IDS_DCU_MSG101_t;

// def @IDS_DCU_MSG102 CAN Message (258  0x102)
#define IDS_DCU_MSG102_IDE (0U)
#define IDS_DCU_MSG102_DLC (8U)
#define IDS_DCU_MSG102_CANID (0x102)
#define IDS_DCU_MSG102_CYC (20U)
// signal: @IDS_DCU_BrkPedCmd_ro
#define TESTDB_IDS_DCU_BrkPedCmd_ro_CovFactor (0.1)
#define TESTDB_IDS_DCU_BrkPedCmd_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_IDS_DCU_BrkPedCmd_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

// Value tables for @IDS_DCU_BrkCtrMode signal

#ifndef IDS_DCU_BrkCtrMode_IDS_DCU_MSG102_
#define IDS_DCU_BrkCtrMode_IDS_DCU_MSG102_ (0)
#endif

#ifndef IDS_DCU_BrkCtrMode_IDS_DCU_MSG102_
#define IDS_DCU_BrkCtrMode_IDS_DCU_MSG102_ (1)
#endif


// Value tables for @IDS_DCU_BrkOverRideDisable signal

#ifndef IDS_DCU_BrkOverRideDisable_IDS_DCU_MSG102_
#define IDS_DCU_BrkOverRideDisable_IDS_DCU_MSG102_ (0)
#endif

#ifndef IDS_DCU_BrkOverRideDisable_IDS_DCU_MSG102_
#define IDS_DCU_BrkOverRideDisable_IDS_DCU_MSG102_ (1)
#endif


// Value tables for @IDS_DCU_BrkCtrlEna signal

#ifndef IDS_DCU_BrkCtrlEna_IDS_DCU_MSG102_
#define IDS_DCU_BrkCtrlEna_IDS_DCU_MSG102_ (0)
#endif

#ifndef IDS_DCU_BrkCtrlEna_IDS_DCU_MSG102_
#define IDS_DCU_BrkCtrlEna_IDS_DCU_MSG102_ (1)
#endif

// signal: @IDS_DCU_BrkPedInv_ro
#define TESTDB_IDS_DCU_BrkPedInv_ro_CovFactor (0.1)
#define TESTDB_IDS_DCU_BrkPedInv_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_IDS_DCU_BrkPedInv_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @IDS_DCU_TargDecelCmd_ro
#define TESTDB_IDS_DCU_TargDecelCmd_ro_CovFactor (0.1)
#define TESTDB_IDS_DCU_TargDecelCmd_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_IDS_DCU_TargDecelCmd_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @IDS_DCU_TargDecelInv_ro
#define TESTDB_IDS_DCU_TargDecelInv_ro_CovFactor (0.1)
#define TESTDB_IDS_DCU_TargDecelInv_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_IDS_DCU_TargDecelInv_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  uint16_t IDS_DCU_BrkPedCmd_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_BrkPedCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "̤�忪�ȿ���"
  //  1 : "���ٶȿ���"
  uint8_t IDS_DCU_BrkCtrMode : 1;            //      Bits= 1 Unit:'-'

  //  0 : "����ɲ���ӹ��˳��Զ�ģʽ"
  //  1 : "ʹ��ɲ���ӹܺ��˳��Զ�ģʽ"
  uint8_t IDS_DCU_BrkOverRideDisable : 1;    //      Bits= 1 Unit:'-'

  //  0 : "����"
  //  1 : "ʹ��"
  uint8_t IDS_DCU_BrkCtrlEna : 1;            //      Bits= 1 Unit:'-'

  uint16_t IDS_DCU_BrkPedInv_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_BrkPedInv_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint8_t IDS_DCU_TargDecelCmd_ro;           //      Bits= 8 Factor= 0.1             Unit:'m/s^2'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_TargDecelCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint8_t IDS_DCU_TargDecelInv_ro;           //      Bits= 8 Factor= 0.1             Unit:'m/s^2'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_TargDecelInv_phys;
#endif // TESTDB_USE_SIGFLOAT

  // ���ļ�����
  uint8_t IDS_DCU_BrkCtrl_CTROLL : 4;        //      Bits= 4 Unit:'-'

  // ����У���
  uint8_t IDS_DCU_BrkCtrl_CHECKSUM;          //      Bits= 8 Unit:'-'

#else

  uint16_t IDS_DCU_BrkPedCmd_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_BrkPedCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "̤�忪�ȿ���"
  //  1 : "���ٶȿ���"
  uint8_t IDS_DCU_BrkCtrMode;                //      Bits= 1 Unit:'-'

  //  0 : "����ɲ���ӹ��˳��Զ�ģʽ"
  //  1 : "ʹ��ɲ���ӹܺ��˳��Զ�ģʽ"
  uint8_t IDS_DCU_BrkOverRideDisable;        //      Bits= 1 Unit:'-'

  //  0 : "����"
  //  1 : "ʹ��"
  uint8_t IDS_DCU_BrkCtrlEna;                //      Bits= 1 Unit:'-'

  uint16_t IDS_DCU_BrkPedInv_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_BrkPedInv_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint8_t IDS_DCU_TargDecelCmd_ro;           //      Bits= 8 Factor= 0.1             Unit:'m/s^2'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_TargDecelCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint8_t IDS_DCU_TargDecelInv_ro;           //      Bits= 8 Factor= 0.1             Unit:'m/s^2'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_TargDecelInv_phys;
#endif // TESTDB_USE_SIGFLOAT

  // ���ļ�����
  uint8_t IDS_DCU_BrkCtrl_CTROLL;            //      Bits= 4 Unit:'-'

  // ����У���
  uint8_t IDS_DCU_BrkCtrl_CHECKSUM;          //      Bits= 8 Unit:'-'

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} IDS_DCU_MSG102_t;

// def @IDS_DCU_MSG103 CAN Message (259  0x103)
#define IDS_DCU_MSG103_IDE (0U)
#define IDS_DCU_MSG103_DLC (8U)
#define IDS_DCU_MSG103_CANID (0x103)
#define IDS_DCU_MSG103_CYC (20U)
// signal: @IDS_DCU_StrAngCmd_ro
#define TESTDB_IDS_DCU_StrAngCmd_ro_CovFactor (0.1)
#define TESTDB_IDS_DCU_StrAngCmd_ro_toS(x) ( (uint16_t) (((x) - (-800.0)) / (0.1)) )
#define TESTDB_IDS_DCU_StrAngCmd_ro_fromS(x) ( (((x) * (0.1)) + (-800.0)) )

// Value tables for @IDS_DCU_StrCtrMode signal

#ifndef IDS_DCU_StrCtrMode_IDS_DCU_MSG103_
#define IDS_DCU_StrCtrMode_IDS_DCU_MSG103_ (0)
#endif

#ifndef IDS_DCU_StrCtrMode_IDS_DCU_MSG103_
#define IDS_DCU_StrCtrMode_IDS_DCU_MSG103_ (1)
#endif


// Value tables for @IDS_DCU_StrCtrlEna signal

#ifndef IDS_DCU_StrCtrlEna_IDS_DCU_MSG103_
#define IDS_DCU_StrCtrlEna_IDS_DCU_MSG103_ (0)
#endif

#ifndef IDS_DCU_StrCtrlEna_IDS_DCU_MSG103_
#define IDS_DCU_StrCtrlEna_IDS_DCU_MSG103_ (1)
#endif

// signal: @IDS_DCU_StrAngLimit_ro
#define TESTDB_IDS_DCU_StrAngLimit_ro_CovFactor (0.1)
#define TESTDB_IDS_DCU_StrAngLimit_ro_toS(x) ( (uint16_t) (((x) - (-800.0)) / (0.1)) )
#define TESTDB_IDS_DCU_StrAngLimit_ro_fromS(x) ( (((x) * (0.1)) + (-800.0)) )
// signal: @IDS_DCU_StrTorq_ro
#define TESTDB_IDS_DCU_StrTorq_ro_CovFactor (0.01)
#define TESTDB_IDS_DCU_StrTorq_ro_toS(x) ( (uint16_t) (((x) - (-10.24)) / (0.01)) )
#define TESTDB_IDS_DCU_StrTorq_ro_fromS(x) ( (((x) * (0.01)) + (-10.24)) )

typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  uint16_t IDS_DCU_StrAngCmd_ro;             //      Bits=14 Offset= -800.0             Factor= 0.1             Unit:'deg'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_StrAngCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "�Ƕȿ���"
  //  1 : "ת�ؿ���"
  uint8_t IDS_DCU_StrCtrMode : 1;            //      Bits= 1 Unit:'-'

  //  0 : "����"
  //  1 : "ʹ��"
  uint8_t IDS_DCU_StrCtrlEna : 1;            //      Bits= 1 Unit:'-'

  uint16_t IDS_DCU_StrAngLimit_ro;           //      Bits=14 Offset= -800.0             Factor= 0.1             Unit:'deg'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_StrAngLimit_phys;
#endif // TESTDB_USE_SIGFLOAT

  // ������ת���ٶ����ƣ���Ч��Χ0��1020deg/s
  uint16_t IDS_DCU_StrAngRateLimit;          //      Bits=12 Unit:'deg/s'

  // ���ļ�����
  uint8_t IDS_DCU_StrCtrl_CTROLL : 4;        //      Bits= 4 Unit:'-'

  uint16_t IDS_DCU_StrTorq_ro;               //      Bits=11 Offset= -10.24             Factor= 0.01            Unit:'Nm'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_StrTorq_phys;
#endif // TESTDB_USE_SIGFLOAT

#else

  uint16_t IDS_DCU_StrAngCmd_ro;             //      Bits=14 Offset= -800.0             Factor= 0.1             Unit:'deg'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_StrAngCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "�Ƕȿ���"
  //  1 : "ת�ؿ���"
  uint8_t IDS_DCU_StrCtrMode;                //      Bits= 1 Unit:'-'

  //  0 : "����"
  //  1 : "ʹ��"
  uint8_t IDS_DCU_StrCtrlEna;                //      Bits= 1 Unit:'-'

  uint16_t IDS_DCU_StrAngLimit_ro;           //      Bits=14 Offset= -800.0             Factor= 0.1             Unit:'deg'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_StrAngLimit_phys;
#endif // TESTDB_USE_SIGFLOAT

  // ������ת���ٶ����ƣ���Ч��Χ0��1020deg/s
  uint16_t IDS_DCU_StrAngRateLimit;          //      Bits=12 Unit:'deg/s'

  // ���ļ�����
  uint8_t IDS_DCU_StrCtrl_CTROLL;            //      Bits= 4 Unit:'-'

  uint16_t IDS_DCU_StrTorq_ro;               //      Bits=11 Offset= -10.24             Factor= 0.01            Unit:'Nm'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t IDS_DCU_StrTorq_phys;
#endif // TESTDB_USE_SIGFLOAT

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} IDS_DCU_MSG103_t;

// def @IDS_DCU_MSG104 CAN Message (260  0x104)
#define IDS_DCU_MSG104_IDE (0U)
#define IDS_DCU_MSG104_DLC (8U)
#define IDS_DCU_MSG104_CANID (0x104)
#define IDS_DCU_MSG104_CYC (20U)

// Value tables for @IDS_DCU_GearCmd signal

#ifndef IDS_DCU_GearCmd_IDS_DCU_MSG104_
#define IDS_DCU_GearCmd_IDS_DCU_MSG104_ (0)
#endif

#ifndef IDS_DCU_GearCmd_IDS_DCU_MSG104_P
#define IDS_DCU_GearCmd_IDS_DCU_MSG104_P (1)
#endif

#ifndef IDS_DCU_GearCmd_IDS_DCU_MSG104_R
#define IDS_DCU_GearCmd_IDS_DCU_MSG104_R (2)
#endif

#ifndef IDS_DCU_GearCmd_IDS_DCU_MSG104_N
#define IDS_DCU_GearCmd_IDS_DCU_MSG104_N (3)
#endif

#ifndef IDS_DCU_GearCmd_IDS_DCU_MSG104_D
#define IDS_DCU_GearCmd_IDS_DCU_MSG104_D (4)
#endif

#ifndef IDS_DCU_GearCmd_IDS_DCU_MSG104_
#define IDS_DCU_GearCmd_IDS_DCU_MSG104_ (5)
#endif


// Value tables for @IDS_DCU_BuzzerCmd signal

#ifndef IDS_DCU_BuzzerCmd_IDS_DCU_MSG104_
#define IDS_DCU_BuzzerCmd_IDS_DCU_MSG104_ (0)
#endif

#ifndef IDS_DCU_BuzzerCmd_IDS_DCU_MSG104_
#define IDS_DCU_BuzzerCmd_IDS_DCU_MSG104_ (1)
#endif


// Value tables for @IDS_DCU_GearCtrlEna signal

#ifndef IDS_DCU_GearCtrlEna_IDS_DCU_MSG104_
#define IDS_DCU_GearCtrlEna_IDS_DCU_MSG104_ (0)
#endif

#ifndef IDS_DCU_GearCtrlEna_IDS_DCU_MSG104_
#define IDS_DCU_GearCtrlEna_IDS_DCU_MSG104_ (1)
#endif


// Value tables for @IDS_DCU_TurnLightCmd signal

#ifndef IDS_DCU_TurnLightCmd_IDS_DCU_MSG104_
#define IDS_DCU_TurnLightCmd_IDS_DCU_MSG104_ (0)
#endif

#ifndef IDS_DCU_TurnLightCmd_IDS_DCU_MSG104_
#define IDS_DCU_TurnLightCmd_IDS_DCU_MSG104_ (1)
#endif

#ifndef IDS_DCU_TurnLightCmd_IDS_DCU_MSG104_
#define IDS_DCU_TurnLightCmd_IDS_DCU_MSG104_ (2)
#endif

#ifndef IDS_DCU_TurnLightCmd_IDS_DCU_MSG104_
#define IDS_DCU_TurnLightCmd_IDS_DCU_MSG104_ (3)
#endif


// Value tables for @IDS_DCU_BeamCmd signal

#ifndef IDS_DCU_BeamCmd_IDS_DCU_MSG104_
#define IDS_DCU_BeamCmd_IDS_DCU_MSG104_ (0)
#endif

#ifndef IDS_DCU_BeamCmd_IDS_DCU_MSG104_
#define IDS_DCU_BeamCmd_IDS_DCU_MSG104_ (1)
#endif

#ifndef IDS_DCU_BeamCmd_IDS_DCU_MSG104_
#define IDS_DCU_BeamCmd_IDS_DCU_MSG104_ (2)
#endif

#ifndef IDS_DCU_BeamCmd_IDS_DCU_MSG104_
#define IDS_DCU_BeamCmd_IDS_DCU_MSG104_ (3)
#endif


// Value tables for @IDS_DCU_EPBCmd signal

#ifndef IDS_DCU_EPBCmd_IDS_DCU_MSG104_
#define IDS_DCU_EPBCmd_IDS_DCU_MSG104_ (0)
#endif

#ifndef IDS_DCU_EPBCmd_IDS_DCU_MSG104_
#define IDS_DCU_EPBCmd_IDS_DCU_MSG104_ (1)
#endif

#ifndef IDS_DCU_EPBCmd_IDS_DCU_MSG104_
#define IDS_DCU_EPBCmd_IDS_DCU_MSG104_ (2)
#endif

#ifndef IDS_DCU_EPBCmd_IDS_DCU_MSG104_
#define IDS_DCU_EPBCmd_IDS_DCU_MSG104_ (3)
#endif


// Value tables for @IDS_DCU_HornCmd signal

#ifndef IDS_DCU_HornCmd_IDS_DCU_MSG104_
#define IDS_DCU_HornCmd_IDS_DCU_MSG104_ (0)
#endif

#ifndef IDS_DCU_HornCmd_IDS_DCU_MSG104_
#define IDS_DCU_HornCmd_IDS_DCU_MSG104_ (1)
#endif


// Value tables for @IDS_ACU_GearManulToAutoMode signal

#ifndef IDS_ACU_GearManulToAutoMode_IDS_DCU_MSG104_D
#define IDS_ACU_GearManulToAutoMode_IDS_DCU_MSG104_D (0)
#endif

#ifndef IDS_ACU_GearManulToAutoMode_IDS_DCU_MSG104_D
#define IDS_ACU_GearManulToAutoMode_IDS_DCU_MSG104_D (1)
#endif


// Value tables for @IDS_DCU_WiperCmd signal

#ifndef IDS_DCU_WiperCmd_IDS_DCU_MSG104__OFF
#define IDS_DCU_WiperCmd_IDS_DCU_MSG104__OFF (0)
#endif

#ifndef IDS_DCU_WiperCmd_IDS_DCU_MSG104__Reserve
#define IDS_DCU_WiperCmd_IDS_DCU_MSG104__Reserve (1)
#endif

#ifndef IDS_DCU_WiperCmd_IDS_DCU_MSG104__SLOW
#define IDS_DCU_WiperCmd_IDS_DCU_MSG104__SLOW (2)
#endif

#ifndef IDS_DCU_WiperCmd_IDS_DCU_MSG104__FAST
#define IDS_DCU_WiperCmd_IDS_DCU_MSG104__FAST (3)
#endif


typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  //  0 : "����"
  //  1 : "P��"
  //  2 : "R��"
  //  3 : "N��"
  //  4 : "D��"
  //  5 : "����"
  uint8_t IDS_DCU_GearCmd : 4;               //      Bits= 4 Unit:'-'

  //  0 : "����"
  //  1 : "�ر�"
  uint8_t IDS_DCU_BuzzerCmd : 1;             //      Bits= 1 Unit:'-'

  //  0 : "����"
  //  1 : "ʹ��"
  uint8_t IDS_DCU_GearCtrlEna : 1;           //      Bits= 1 Unit:'-'

  //  0 : "������ת���"
  //  1 : "��ת�����˸"
  //  2 : "��ת�����˸"
  //  3 : "˫��"
  uint8_t IDS_DCU_TurnLightCmd : 2;          //      Bits= 2 Unit:'-'

  //  0 : "������"
  //  1 : "�������"
  //  2 : "��Զ���"
  //  3 : "����"
  uint8_t IDS_DCU_BeamCmd : 2;               //      Bits= 2 Unit:'-'

  //  0 : "�޶���"
  //  1 : "��ɲ����"
  //  2 : "��ɲ�ͷ�"
  //  3 : "����"
  uint8_t IDS_DCU_EPBCmd : 2;                //      Bits= 2 Unit:'-'

  //  0 : "������"
  //  1 : "����"
  uint8_t IDS_DCU_HornCmd : 1;               //      Bits= 1 Unit:'-'

  //  0 : "D�����ܽ��߿�"
  //  1 : "ʹ��D�����߿�"
  uint8_t IDS_ACU_GearManulToAutoMode : 1;   //      Bits= 1 Unit:'-'

  uint8_t IDS_DCU_Gear_CTROLL : 4;           //      Bits= 4 Unit:'-'

  //  0 : " OFF"
  //  1 : " Reserve"
  //  2 : " SLOW"
  //  3 : " FAST"
  uint8_t IDS_DCU_WiperCmd : 2;              //      Bits= 2 Unit:'-'

  uint8_t IDS_DCU_Gear_CHECKSUM;             //      Bits= 8 Unit:'-'

#else

  //  0 : "����"
  //  1 : "P��"
  //  2 : "R��"
  //  3 : "N��"
  //  4 : "D��"
  //  5 : "����"
  uint8_t IDS_DCU_GearCmd;                   //      Bits= 4 Unit:'-'

  //  0 : "����"
  //  1 : "�ر�"
  uint8_t IDS_DCU_BuzzerCmd;                 //      Bits= 1 Unit:'-'

  //  0 : "����"
  //  1 : "ʹ��"
  uint8_t IDS_DCU_GearCtrlEna;               //      Bits= 1 Unit:'-'

  //  0 : "������ת���"
  //  1 : "��ת�����˸"
  //  2 : "��ת�����˸"
  //  3 : "˫��"
  uint8_t IDS_DCU_TurnLightCmd;              //      Bits= 2 Unit:'-'

  //  0 : "������"
  //  1 : "�������"
  //  2 : "��Զ���"
  //  3 : "����"
  uint8_t IDS_DCU_BeamCmd;                   //      Bits= 2 Unit:'-'

  //  0 : "�޶���"
  //  1 : "��ɲ����"
  //  2 : "��ɲ�ͷ�"
  //  3 : "����"
  uint8_t IDS_DCU_EPBCmd;                    //      Bits= 2 Unit:'-'

  //  0 : "������"
  //  1 : "����"
  uint8_t IDS_DCU_HornCmd;                   //      Bits= 1 Unit:'-'

  //  0 : "D�����ܽ��߿�"
  //  1 : "ʹ��D�����߿�"
  uint8_t IDS_ACU_GearManulToAutoMode;       //      Bits= 1 Unit:'-'

  uint8_t IDS_DCU_Gear_CTROLL;               //      Bits= 4 Unit:'-'

  //  0 : " OFF"
  //  1 : " Reserve"
  //  2 : " SLOW"
  //  3 : " FAST"
  uint8_t IDS_DCU_WiperCmd;                  //      Bits= 2 Unit:'-'

  uint8_t IDS_DCU_Gear_CHECKSUM;             //      Bits= 8 Unit:'-'

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} IDS_DCU_MSG104_t;

// def @DCU_IDS_MSG200 CAN Message (512  0x200)
#define DCU_IDS_MSG200_IDE (0U)
#define DCU_IDS_MSG200_DLC (8U)
#define DCU_IDS_MSG200_CANID (0x200)
#define DCU_IDS_MSG200_CYC (20U)

// Value tables for @DCU_IDS_AutoCtrlStat signal

#ifndef DCU_IDS_AutoCtrlStat_DCU_IDS_MSG200_
#define DCU_IDS_AutoCtrlStat_DCU_IDS_MSG200_ (0)
#endif

#ifndef DCU_IDS_AutoCtrlStat_DCU_IDS_MSG200_
#define DCU_IDS_AutoCtrlStat_DCU_IDS_MSG200_ (1)
#endif

#ifndef DCU_IDS_AutoCtrlStat_DCU_IDS_MSG200_
#define DCU_IDS_AutoCtrlStat_DCU_IDS_MSG200_ (2)
#endif


// Value tables for @DCU_IDS_FRWheelRotatedDirection signal

#ifndef DCU_IDS_FRWheelRotatedDirection_DCU_IDS_MSG200_
#define DCU_IDS_FRWheelRotatedDirection_DCU_IDS_MSG200_ (0)
#endif

#ifndef DCU_IDS_FRWheelRotatedDirection_DCU_IDS_MSG200_
#define DCU_IDS_FRWheelRotatedDirection_DCU_IDS_MSG200_ (1)
#endif


// Value tables for @DCU_IDS_FLWheelRotatedDirection signal

#ifndef DCU_IDS_FLWheelRotatedDirection_DCU_IDS_MSG200_
#define DCU_IDS_FLWheelRotatedDirection_DCU_IDS_MSG200_ (0)
#endif

#ifndef DCU_IDS_FLWheelRotatedDirection_DCU_IDS_MSG200_
#define DCU_IDS_FLWheelRotatedDirection_DCU_IDS_MSG200_ (1)
#endif


// Value tables for @DCU_IDS_RRWheelRotatedDirection signal

#ifndef DCU_IDS_RRWheelRotatedDirection_DCU_IDS_MSG200_
#define DCU_IDS_RRWheelRotatedDirection_DCU_IDS_MSG200_ (0)
#endif

#ifndef DCU_IDS_RRWheelRotatedDirection_DCU_IDS_MSG200_
#define DCU_IDS_RRWheelRotatedDirection_DCU_IDS_MSG200_ (1)
#endif


// Value tables for @DCU_IDS_RLWheelRotatedDirection signal

#ifndef DCU_IDS_RLWheelRotatedDirection_DCU_IDS_MSG200_
#define DCU_IDS_RLWheelRotatedDirection_DCU_IDS_MSG200_ (0)
#endif

#ifndef DCU_IDS_RLWheelRotatedDirection_DCU_IDS_MSG200_
#define DCU_IDS_RLWheelRotatedDirection_DCU_IDS_MSG200_ (1)
#endif


// Value tables for @DCU_IDS_bBrkTakeover signal

#ifndef DCU_IDS_bBrkTakeover_DCU_IDS_MSG200__
#define DCU_IDS_bBrkTakeover_DCU_IDS_MSG200__ (0)
#endif

#ifndef DCU_IDS_bBrkTakeover_DCU_IDS_MSG200__
#define DCU_IDS_bBrkTakeover_DCU_IDS_MSG200__ (1)
#endif


// Value tables for @DCU_IDS_bAccTakeover signal

#ifndef DCU_IDS_bAccTakeover_DCU_IDS_MSG200__
#define DCU_IDS_bAccTakeover_DCU_IDS_MSG200__ (0)
#endif

#ifndef DCU_IDS_bAccTakeover_DCU_IDS_MSG200__
#define DCU_IDS_bAccTakeover_DCU_IDS_MSG200__ (1)
#endif


// Value tables for @DCU_IDS_bStrTakeover signal

#ifndef DCU_IDS_bStrTakeover_DCU_IDS_MSG200__
#define DCU_IDS_bStrTakeover_DCU_IDS_MSG200__ (0)
#endif

#ifndef DCU_IDS_bStrTakeover_DCU_IDS_MSG200__
#define DCU_IDS_bStrTakeover_DCU_IDS_MSG200__ (1)
#endif


// Value tables for @DCU_IDS_bOtherTakeover signal

#ifndef DCU_IDS_bOtherTakeover_DCU_IDS_MSG200__
#define DCU_IDS_bOtherTakeover_DCU_IDS_MSG200__ (0)
#endif

#ifndef DCU_IDS_bOtherTakeover_DCU_IDS_MSG200__
#define DCU_IDS_bOtherTakeover_DCU_IDS_MSG200__ (1)
#endif


typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  //  0 : "�ֶ���ʻģʽ"
  //  1 : "�Զ���ʻģʽ"
  //  2 : "����ģʽ"
  uint8_t DCU_IDS_AutoCtrlStat : 2;              //      Bits= 2 Unit:'-'

  //  0 : "ǰ��"
  //  1 : "����"
  uint8_t DCU_IDS_FRWheelRotatedDirection : 1;   //      Bits= 1

  //  0 : "ǰ��"
  //  1 : "����"
  uint8_t DCU_IDS_FLWheelRotatedDirection : 1;   //      Bits= 1

  //  0 : "ǰ��"
  //  1 : "����"
  uint8_t DCU_IDS_RRWheelRotatedDirection : 1;   //      Bits= 1

  //  0 : "ǰ��"
  //  1 : "����"
  uint8_t DCU_IDS_RLWheelRotatedDirection : 1;   //      Bits= 1

  // ��������
  uint8_t DCU_IDS_WhlPlsFr;                      //      Bits= 8 Unit:'-'

  // ��������
  uint8_t DCU_IDS_WhlPlsFl;                      //      Bits= 8 Unit:'-'

  // ��������
  uint8_t DCU_IDS_WhlPlsRl;                      //      Bits= 8 Unit:'-'

  // ��������
  uint8_t DCU_IDS_WhlPlsRr;                      //      Bits= 8 Unit:'-'

  // ���ļ�����
  uint8_t DCU_IDS_ModeStat_CTROLL : 4;           //      Bits= 4 Unit:'-'

  //  0 : " δ�ӹ�"
  //  1 : " ɲ���ӹ�"
  uint8_t DCU_IDS_bBrkTakeover : 1;              //      Bits= 1 Unit:'-'

  //  0 : " δ�ӹ�"
  //  1 : " ���Žӹ�"
  uint8_t DCU_IDS_bAccTakeover : 1;              //      Bits= 1 Unit:'-'

  //  0 : " δ�ӹ�"
  //  1 : " ת��ӹ�"
  uint8_t DCU_IDS_bStrTakeover : 1;              //      Bits= 1 Unit:'-'

  //  0 : " δ�ӹ�"
  //  1 : " ����ԭ��ӹ�"
  uint8_t DCU_IDS_bOtherTakeover : 1;            //      Bits= 1 Unit:'-'

  // ����У���
  uint8_t DCU_IDS_ModeStat_CHECKSUM;             //      Bits= 8 Unit:'-'

#else

  //  0 : "�ֶ���ʻģʽ"
  //  1 : "�Զ���ʻģʽ"
  //  2 : "����ģʽ"
  uint8_t DCU_IDS_AutoCtrlStat;                  //      Bits= 2 Unit:'-'

  //  0 : "ǰ��"
  //  1 : "����"
  uint8_t DCU_IDS_FRWheelRotatedDirection;       //      Bits= 1

  //  0 : "ǰ��"
  //  1 : "����"
  uint8_t DCU_IDS_FLWheelRotatedDirection;       //      Bits= 1

  //  0 : "ǰ��"
  //  1 : "����"
  uint8_t DCU_IDS_RRWheelRotatedDirection;       //      Bits= 1

  //  0 : "ǰ��"
  //  1 : "����"
  uint8_t DCU_IDS_RLWheelRotatedDirection;       //      Bits= 1

  // ��������
  uint8_t DCU_IDS_WhlPlsFr;                      //      Bits= 8 Unit:'-'

  // ��������
  uint8_t DCU_IDS_WhlPlsFl;                      //      Bits= 8 Unit:'-'

  // ��������
  uint8_t DCU_IDS_WhlPlsRl;                      //      Bits= 8 Unit:'-'

  // ��������
  uint8_t DCU_IDS_WhlPlsRr;                      //      Bits= 8 Unit:'-'

  // ���ļ�����
  uint8_t DCU_IDS_ModeStat_CTROLL;               //      Bits= 4 Unit:'-'

  //  0 : " δ�ӹ�"
  //  1 : " ɲ���ӹ�"
  uint8_t DCU_IDS_bBrkTakeover;                  //      Bits= 1 Unit:'-'

  //  0 : " δ�ӹ�"
  //  1 : " ���Žӹ�"
  uint8_t DCU_IDS_bAccTakeover;                  //      Bits= 1 Unit:'-'

  //  0 : " δ�ӹ�"
  //  1 : " ת��ӹ�"
  uint8_t DCU_IDS_bStrTakeover;                  //      Bits= 1 Unit:'-'

  //  0 : " δ�ӹ�"
  //  1 : " ����ԭ��ӹ�"
  uint8_t DCU_IDS_bOtherTakeover;                //      Bits= 1 Unit:'-'

  // ����У���
  uint8_t DCU_IDS_ModeStat_CHECKSUM;             //      Bits= 8 Unit:'-'

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} DCU_IDS_MSG200_t;

// def @DCU_IDS_MSG201 CAN Message (513  0x201)
#define DCU_IDS_MSG201_IDE (0U)
#define DCU_IDS_MSG201_DLC (8U)
#define DCU_IDS_MSG201_CANID (0x201)
#define DCU_IDS_MSG201_CYC (20U)
// signal: @DCU_IDS_AccPedAct_ro
#define TESTDB_DCU_IDS_AccPedAct_ro_CovFactor (0.1)
#define TESTDB_DCU_IDS_AccPedAct_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_DCU_IDS_AccPedAct_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

// Value tables for @DCU_IDS_AccCtrlModeSt signal

#ifndef DCU_IDS_AccCtrlModeSt_DCU_IDS_MSG201_
#define DCU_IDS_AccCtrlModeSt_DCU_IDS_MSG201_ (0)
#endif

#ifndef DCU_IDS_AccCtrlModeSt_DCU_IDS_MSG201_
#define DCU_IDS_AccCtrlModeSt_DCU_IDS_MSG201_ (1)
#endif

#ifndef DCU_IDS_AccCtrlModeSt_DCU_IDS_MSG201_
#define DCU_IDS_AccCtrlModeSt_DCU_IDS_MSG201_ (2)
#endif


// Value tables for @DCU_IDS_AccCtrlStat signal

#ifndef DCU_IDS_AccCtrlStat_DCU_IDS_MSG201_
#define DCU_IDS_AccCtrlStat_DCU_IDS_MSG201_ (0)
#endif

#ifndef DCU_IDS_AccCtrlStat_DCU_IDS_MSG201_
#define DCU_IDS_AccCtrlStat_DCU_IDS_MSG201_ (1)
#endif

#ifndef DCU_IDS_AccCtrlStat_DCU_IDS_MSG201_
#define DCU_IDS_AccCtrlStat_DCU_IDS_MSG201_ (2)
#endif

// signal: @DCU_IDS_AccPedCmd_ro
#define TESTDB_DCU_IDS_AccPedCmd_ro_CovFactor (0.1)
#define TESTDB_DCU_IDS_AccPedCmd_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_DCU_IDS_AccPedCmd_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @DCU_IDS_AccPedExe_ro
#define TESTDB_DCU_IDS_AccPedExe_ro_CovFactor (0.1)
#define TESTDB_DCU_IDS_AccPedExe_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_DCU_IDS_AccPedExe_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  uint16_t DCU_IDS_AccPedAct_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_AccPedAct_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "δʹ��"
  //  1 : "���ſ��ȿ���"
  //  2 : "�ֶ�Ť�ؿ���"
  uint8_t DCU_IDS_AccCtrlModeSt : 2;         //      Bits= 2 Unit:'-'

  //  0 : "�ֶ�����ģʽ"
  //  1 : "�Զ�����ģʽ"
  //  2 : "����̤����������У�����"
  uint8_t DCU_IDS_AccCtrlStat : 2;           //      Bits= 2 Unit:'-'

  uint16_t DCU_IDS_AccPedCmd_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_AccPedCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_AccPedExe_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_AccPedExe_phys;
#endif // TESTDB_USE_SIGFLOAT

  // ʵ���ֶ�Ť��
  uint16_t DCU_IDS_ActVehWheelTorq;          //      Bits=14 Unit:'Nm'

#else

  uint16_t DCU_IDS_AccPedAct_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_AccPedAct_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "δʹ��"
  //  1 : "���ſ��ȿ���"
  //  2 : "�ֶ�Ť�ؿ���"
  uint8_t DCU_IDS_AccCtrlModeSt;             //      Bits= 2 Unit:'-'

  //  0 : "�ֶ�����ģʽ"
  //  1 : "�Զ�����ģʽ"
  //  2 : "����̤����������У�����"
  uint8_t DCU_IDS_AccCtrlStat;               //      Bits= 2 Unit:'-'

  uint16_t DCU_IDS_AccPedCmd_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_AccPedCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_AccPedExe_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_AccPedExe_phys;
#endif // TESTDB_USE_SIGFLOAT

  // ʵ���ֶ�Ť��
  uint16_t DCU_IDS_ActVehWheelTorq;          //      Bits=14 Unit:'Nm'

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} DCU_IDS_MSG201_t;

// def @DCU_IDS_MSG202 CAN Message (514  0x202)
#define DCU_IDS_MSG202_IDE (0U)
#define DCU_IDS_MSG202_DLC (8U)
#define DCU_IDS_MSG202_CANID (0x202)
#define DCU_IDS_MSG202_CYC (20U)
// signal: @DCU_IDS_BrkPedAct_ro
#define TESTDB_DCU_IDS_BrkPedAct_ro_CovFactor (0.1)
#define TESTDB_DCU_IDS_BrkPedAct_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_DCU_IDS_BrkPedAct_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

// Value tables for @DCU_IDS_BrkCtrlModeSt signal

#ifndef DCU_IDS_BrkCtrlModeSt_DCU_IDS_MSG202_
#define DCU_IDS_BrkCtrlModeSt_DCU_IDS_MSG202_ (0)
#endif

#ifndef DCU_IDS_BrkCtrlModeSt_DCU_IDS_MSG202_
#define DCU_IDS_BrkCtrlModeSt_DCU_IDS_MSG202_ (1)
#endif

#ifndef DCU_IDS_BrkCtrlModeSt_DCU_IDS_MSG202_
#define DCU_IDS_BrkCtrlModeSt_DCU_IDS_MSG202_ (2)
#endif


// Value tables for @DCU_IDS_BrkCtrlStat signal

#ifndef DCU_IDS_BrkCtrlStat_DCU_IDS_MSG202_
#define DCU_IDS_BrkCtrlStat_DCU_IDS_MSG202_ (0)
#endif

#ifndef DCU_IDS_BrkCtrlStat_DCU_IDS_MSG202_
#define DCU_IDS_BrkCtrlStat_DCU_IDS_MSG202_ (1)
#endif

#ifndef DCU_IDS_BrkCtrlStat_DCU_IDS_MSG202_
#define DCU_IDS_BrkCtrlStat_DCU_IDS_MSG202_ (2)
#endif

// signal: @DCU_IDS_BrkPedCmd_ro
#define TESTDB_DCU_IDS_BrkPedCmd_ro_CovFactor (0.1)
#define TESTDB_DCU_IDS_BrkPedCmd_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_DCU_IDS_BrkPedCmd_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @DCU_IDS_BrkPedExe_ro
#define TESTDB_DCU_IDS_BrkPedExe_ro_CovFactor (0.1)
#define TESTDB_DCU_IDS_BrkPedExe_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define TESTDB_DCU_IDS_BrkPedExe_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  uint16_t DCU_IDS_BrkPedAct_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_BrkPedAct_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "δʹ��"
  //  1 : "̤�忪�ȿ���"
  //  2 : "���ٶȿ���"
  uint8_t DCU_IDS_BrkCtrlModeSt : 2;         //      Bits= 2 Unit:'-'

  //  0 : "�ֶ�����ģʽ"
  //  1 : "�Զ�����ģʽ"
  //  2 : "�ƶ�̤����������У�����"
  uint8_t DCU_IDS_BrkCtrlStat : 2;           //      Bits= 2 Unit:'-'

  uint16_t DCU_IDS_BrkPedCmd_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_BrkPedCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_BrkPedExe_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_BrkPedExe_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint8_t DCU_IDS_BrkStat_CTROLL : 4;        //      Bits= 4 Unit:'-'

  uint8_t DCU_IDS_BrkStat_CHECKSUM;          //      Bits= 8 Unit:'-'

#else

  uint16_t DCU_IDS_BrkPedAct_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_BrkPedAct_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "δʹ��"
  //  1 : "̤�忪�ȿ���"
  //  2 : "���ٶȿ���"
  uint8_t DCU_IDS_BrkCtrlModeSt;             //      Bits= 2 Unit:'-'

  //  0 : "�ֶ�����ģʽ"
  //  1 : "�Զ�����ģʽ"
  //  2 : "�ƶ�̤����������У�����"
  uint8_t DCU_IDS_BrkCtrlStat;               //      Bits= 2 Unit:'-'

  uint16_t DCU_IDS_BrkPedCmd_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_BrkPedCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_BrkPedExe_ro;             //      Bits=10 Factor= 0.1             Unit:'%'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_BrkPedExe_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint8_t DCU_IDS_BrkStat_CTROLL;            //      Bits= 4 Unit:'-'

  uint8_t DCU_IDS_BrkStat_CHECKSUM;          //      Bits= 8 Unit:'-'

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} DCU_IDS_MSG202_t;

// def @DCU_IDS_MSG203 CAN Message (515  0x203)
#define DCU_IDS_MSG203_IDE (0U)
#define DCU_IDS_MSG203_DLC (8U)
#define DCU_IDS_MSG203_CANID (0x203)
#define DCU_IDS_MSG203_CYC (20U)
// signal: @DCU_IDS_StrAngAct_ro
#define TESTDB_DCU_IDS_StrAngAct_ro_CovFactor (0.1)
#define TESTDB_DCU_IDS_StrAngAct_ro_toS(x) ( (uint16_t) (((x) - (-800.0)) / (0.1)) )
#define TESTDB_DCU_IDS_StrAngAct_ro_fromS(x) ( (((x) * (0.1)) + (-800.0)) )

// Value tables for @DCU_IDS_StrCtrlStat signal

#ifndef DCU_IDS_StrCtrlStat_DCU_IDS_MSG203_
#define DCU_IDS_StrCtrlStat_DCU_IDS_MSG203_ (0)
#endif

#ifndef DCU_IDS_StrCtrlStat_DCU_IDS_MSG203_
#define DCU_IDS_StrCtrlStat_DCU_IDS_MSG203_ (1)
#endif

#ifndef DCU_IDS_StrCtrlStat_DCU_IDS_MSG203__
#define DCU_IDS_StrCtrlStat_DCU_IDS_MSG203__ (2)
#endif

// signal: @DCU_IDS_StrAngCmd_ro
#define TESTDB_DCU_IDS_StrAngCmd_ro_CovFactor (0.1)
#define TESTDB_DCU_IDS_StrAngCmd_ro_toS(x) ( (uint16_t) (((x) - (-800.0)) / (0.1)) )
#define TESTDB_DCU_IDS_StrAngCmd_ro_fromS(x) ( (((x) * (0.1)) + (-800.0)) )

// Value tables for @DCU_IDS_bEpsCtrFailed signal

#ifndef DCU_IDS_bEpsCtrFailed_DCU_IDS_MSG203__EPS_Failed0
#define DCU_IDS_bEpsCtrFailed_DCU_IDS_MSG203__EPS_Failed0 (1)
#endif

// signal: @DCU_IDS_StrTrqAct_ro
#define TESTDB_DCU_IDS_StrTrqAct_ro_CovFactor (0.1)
#define TESTDB_DCU_IDS_StrTrqAct_ro_toS(x) ( (uint8_t) (((x) - (-12.7)) / (0.1)) )
#define TESTDB_DCU_IDS_StrTrqAct_ro_fromS(x) ( (((x) * (0.1)) + (-12.7)) )
// signal: @DCU_IDS_SteeringAngleSpd_ro
#define TESTDB_DCU_IDS_SteeringAngleSpd_ro_CovFactor (4)
#define TESTDB_DCU_IDS_SteeringAngleSpd_ro_toS(x) ( (uint8_t) ((x) / (4)) )
#define TESTDB_DCU_IDS_SteeringAngleSpd_ro_fromS(x) ( ((x) * (4)) )

typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  uint16_t DCU_IDS_StrAngAct_ro;             //      Bits=14 Offset= -800.0             Factor= 0.1             Unit:'deg'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_StrAngAct_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "ת���ֶ�����"
  //  1 : "ת���Զ�����"
  //  2 : " ת�������ȹ���"
  uint8_t DCU_IDS_StrCtrlStat : 2;           //      Bits= 2 Unit:'-'

  uint16_t DCU_IDS_StrAngCmd_ro;             //      Bits=14 Offset= -800.0             Factor= 0.1             Unit:'deg'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_StrAngCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  // EPSʧ���ź�1: EPS Failed
  // 0: Normal St
  //  1 : " EPS Failed0"
  uint8_t DCU_IDS_bEpsCtrFailed : 1;         //      Bits= 1 Unit:'-'

  uint8_t DCU_IDS_StrTrqAct_ro;              //      Bits= 8 Offset= -12.7              Factor= 0.1             Unit:'Nm'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_StrTrqAct_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint8_t DCU_IDS_SteeringAngleSpd_ro;       //      Bits= 8 Factor= 4               Unit:'deg/sec'

#ifdef TESTDB_USE_SIGFLOAT
  uint16_t DCU_IDS_SteeringAngleSpd_phys;
#endif // TESTDB_USE_SIGFLOAT

  // ���ļ�����
  uint8_t DCU_IDS_StrStat_CTROLL : 4;        //      Bits= 4 Unit:'-'

  // ����У���
  uint8_t DCU_IDS_StrStat_CHECKSUM;          //      Bits= 8 Unit:'-'

#else

  uint16_t DCU_IDS_StrAngAct_ro;             //      Bits=14 Offset= -800.0             Factor= 0.1             Unit:'deg'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_StrAngAct_phys;
#endif // TESTDB_USE_SIGFLOAT

  //  0 : "ת���ֶ�����"
  //  1 : "ת���Զ�����"
  //  2 : " ת�������ȹ���"
  uint8_t DCU_IDS_StrCtrlStat;               //      Bits= 2 Unit:'-'

  uint16_t DCU_IDS_StrAngCmd_ro;             //      Bits=14 Offset= -800.0             Factor= 0.1             Unit:'deg'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_StrAngCmd_phys;
#endif // TESTDB_USE_SIGFLOAT

  // EPSʧ���ź�1: EPS Failed
  // 0: Normal St
  //  1 : " EPS Failed0"
  uint8_t DCU_IDS_bEpsCtrFailed;             //      Bits= 1 Unit:'-'

  uint8_t DCU_IDS_StrTrqAct_ro;              //      Bits= 8 Offset= -12.7              Factor= 0.1             Unit:'Nm'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_StrTrqAct_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint8_t DCU_IDS_SteeringAngleSpd_ro;       //      Bits= 8 Factor= 4               Unit:'deg/sec'

#ifdef TESTDB_USE_SIGFLOAT
  uint16_t DCU_IDS_SteeringAngleSpd_phys;
#endif // TESTDB_USE_SIGFLOAT

  // ���ļ�����
  uint8_t DCU_IDS_StrStat_CTROLL;            //      Bits= 4 Unit:'-'

  // ����У���
  uint8_t DCU_IDS_StrStat_CHECKSUM;          //      Bits= 8 Unit:'-'

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} DCU_IDS_MSG203_t;

// def @DCU_IDS_MSG204 CAN Message (516  0x204)
#define DCU_IDS_MSG204_IDE (0U)
#define DCU_IDS_MSG204_DLC (8U)
#define DCU_IDS_MSG204_CANID (0x204)
#define DCU_IDS_MSG204_CYC (100U)

// Value tables for @DCU_IDS_GearAct signal

#ifndef DCU_IDS_GearAct_DCU_IDS_MSG204__P
#define DCU_IDS_GearAct_DCU_IDS_MSG204__P (1)
#endif

#ifndef DCU_IDS_GearAct_DCU_IDS_MSG204__R
#define DCU_IDS_GearAct_DCU_IDS_MSG204__R (2)
#endif

#ifndef DCU_IDS_GearAct_DCU_IDS_MSG204__N
#define DCU_IDS_GearAct_DCU_IDS_MSG204__N (3)
#endif

#ifndef DCU_IDS_GearAct_DCU_IDS_MSG204__D
#define DCU_IDS_GearAct_DCU_IDS_MSG204__D (4)
#endif


// Value tables for @DCU_IDS_GearCmd signal

#ifndef DCU_IDS_GearCmd_DCU_IDS_MSG204__
#define DCU_IDS_GearCmd_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_GearCmd_DCU_IDS_MSG204__P
#define DCU_IDS_GearCmd_DCU_IDS_MSG204__P (1)
#endif

#ifndef DCU_IDS_GearCmd_DCU_IDS_MSG204__R
#define DCU_IDS_GearCmd_DCU_IDS_MSG204__R (2)
#endif

#ifndef DCU_IDS_GearCmd_DCU_IDS_MSG204__N
#define DCU_IDS_GearCmd_DCU_IDS_MSG204__N (3)
#endif

#ifndef DCU_IDS_GearCmd_DCU_IDS_MSG204__D
#define DCU_IDS_GearCmd_DCU_IDS_MSG204__D (4)
#endif

#ifndef DCU_IDS_GearCmd_DCU_IDS_MSG204__
#define DCU_IDS_GearCmd_DCU_IDS_MSG204__ (5)
#endif


// Value tables for @DCU_IDS_TurnLightCmd signal

#ifndef DCU_IDS_TurnLightCmd_DCU_IDS_MSG204__
#define DCU_IDS_TurnLightCmd_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_TurnLightCmd_DCU_IDS_MSG204__
#define DCU_IDS_TurnLightCmd_DCU_IDS_MSG204__ (1)
#endif

#ifndef DCU_IDS_TurnLightCmd_DCU_IDS_MSG204__
#define DCU_IDS_TurnLightCmd_DCU_IDS_MSG204__ (2)
#endif

#ifndef DCU_IDS_TurnLightCmd_DCU_IDS_MSG204__
#define DCU_IDS_TurnLightCmd_DCU_IDS_MSG204__ (3)
#endif


// Value tables for @DCU_IDS_TurnLightAct signal

#ifndef DCU_IDS_TurnLightAct_DCU_IDS_MSG204__
#define DCU_IDS_TurnLightAct_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_TurnLightAct_DCU_IDS_MSG204__
#define DCU_IDS_TurnLightAct_DCU_IDS_MSG204__ (1)
#endif

#ifndef DCU_IDS_TurnLightAct_DCU_IDS_MSG204__
#define DCU_IDS_TurnLightAct_DCU_IDS_MSG204__ (2)
#endif

#ifndef DCU_IDS_TurnLightAct_DCU_IDS_MSG204__
#define DCU_IDS_TurnLightAct_DCU_IDS_MSG204__ (3)
#endif


// Value tables for @DCU_IDS_CruiseOnBtn signal

#ifndef DCU_IDS_CruiseOnBtn_DCU_IDS_MSG204__
#define DCU_IDS_CruiseOnBtn_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_CruiseOnBtn_DCU_IDS_MSG204__
#define DCU_IDS_CruiseOnBtn_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_CruiseCnclBtn signal

#ifndef DCU_IDS_CruiseCnclBtn_DCU_IDS_MSG204__
#define DCU_IDS_CruiseCnclBtn_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_CruiseCnclBtn_DCU_IDS_MSG204__
#define DCU_IDS_CruiseCnclBtn_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_LowBeamAct signal

#ifndef DCU_IDS_LowBeamAct_DCU_IDS_MSG204__
#define DCU_IDS_LowBeamAct_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_LowBeamAct_DCU_IDS_MSG204__
#define DCU_IDS_LowBeamAct_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_HighBeamAct signal

#ifndef DCU_IDS_HighBeamAct_DCU_IDS_MSG204__
#define DCU_IDS_HighBeamAct_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_HighBeamAct_DCU_IDS_MSG204__
#define DCU_IDS_HighBeamAct_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_EmLightAct signal

#ifndef DCU_IDS_EmLightAct_DCU_IDS_MSG204__
#define DCU_IDS_EmLightAct_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_EmLightAct_DCU_IDS_MSG204__
#define DCU_IDS_EmLightAct_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_CruiseResBtn signal

#ifndef DCU_IDS_CruiseResBtn_DCU_IDS_MSG204__
#define DCU_IDS_CruiseResBtn_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_CruiseResBtn_DCU_IDS_MSG204__
#define DCU_IDS_CruiseResBtn_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_CruiseSetBtn signal

#ifndef DCU_IDS_CruiseSetBtn_DCU_IDS_MSG204__
#define DCU_IDS_CruiseSetBtn_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_CruiseSetBtn_DCU_IDS_MSG204__
#define DCU_IDS_CruiseSetBtn_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_CruiseDisPls signal

#ifndef DCU_IDS_CruiseDisPls_DCU_IDS_MSG204__
#define DCU_IDS_CruiseDisPls_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_CruiseDisPls_DCU_IDS_MSG204__
#define DCU_IDS_CruiseDisPls_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_HornStatus signal

#ifndef DCU_IDS_HornStatus_DCU_IDS_MSG204__
#define DCU_IDS_HornStatus_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_HornStatus_DCU_IDS_MSG204__
#define DCU_IDS_HornStatus_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_EPBStatus signal

#ifndef DCU_IDS_EPBStatus_DCU_IDS_MSG204__
#define DCU_IDS_EPBStatus_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_EPBStatus_DCU_IDS_MSG204__
#define DCU_IDS_EPBStatus_DCU_IDS_MSG204__ (1)
#endif

#ifndef DCU_IDS_EPBStatus_DCU_IDS_MSG204__
#define DCU_IDS_EPBStatus_DCU_IDS_MSG204__ (2)
#endif

#ifndef DCU_IDS_EPBStatus_DCU_IDS_MSG204__
#define DCU_IDS_EPBStatus_DCU_IDS_MSG204__ (3)
#endif


// Value tables for @DCU_IDS_RearFogLampAct signal

#ifndef DCU_IDS_RearFogLampAct_DCU_IDS_MSG204__
#define DCU_IDS_RearFogLampAct_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_RearFogLampAct_DCU_IDS_MSG204__
#define DCU_IDS_RearFogLampAct_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_FrontFogLampAct signal

#ifndef DCU_IDS_FrontFogLampAct_DCU_IDS_MSG204__
#define DCU_IDS_FrontFogLampAct_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_FrontFogLampAct_DCU_IDS_MSG204__
#define DCU_IDS_FrontFogLampAct_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_LKSBtnInfo signal

#ifndef DCU_IDS_LKSBtnInfo_DCU_IDS_MSG204__
#define DCU_IDS_LKSBtnInfo_DCU_IDS_MSG204__ (0)
#endif

#ifndef DCU_IDS_LKSBtnInfo_DCU_IDS_MSG204__
#define DCU_IDS_LKSBtnInfo_DCU_IDS_MSG204__ (1)
#endif


// Value tables for @DCU_IDS_BuzzerCmd signal

#ifndef DCU_IDS_BuzzerCmd_DCU_IDS_MSG204_
#define DCU_IDS_BuzzerCmd_DCU_IDS_MSG204_ (0)
#endif

#ifndef DCU_IDS_BuzzerCmd_DCU_IDS_MSG204_
#define DCU_IDS_BuzzerCmd_DCU_IDS_MSG204_ (1)
#endif


typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  //  1 : " P��"
  //  2 : " R��"
  //  3 : " N��"
  //  4 : " D��"
  uint8_t DCU_IDS_GearAct : 4;               //      Bits= 4

  //  0 : " ����"
  //  1 : " P��"
  //  2 : " R��"
  //  3 : " N��"
  //  4 : " D��"
  //  5 : " ����"
  uint8_t DCU_IDS_GearCmd : 4;               //      Bits= 4

  //  0 : " ������ת���"
  //  1 : " ��ת�����˸"
  //  2 : " ��ת�����˸"
  //  3 : " ˫��"
  uint8_t DCU_IDS_TurnLightCmd : 2;          //      Bits= 2

  //  0 : " ������ת���"
  //  1 : " ��ת�����˸"
  //  2 : " ��ת�����˸"
  //  3 : " ����"
  uint8_t DCU_IDS_TurnLightAct : 2;          //      Bits= 2

  // ����ACC���ܰ���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_CruiseOnBtn : 1;           //      Bits= 1

  // �ر�ACC���ܰ���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_CruiseCnclBtn : 1;         //      Bits= 1

  //  0 : " ����ƹر�"
  //  1 : " ����ƿ���"
  uint8_t DCU_IDS_LowBeamAct : 1;            //      Bits= 1

  //  0 : " Զ��ƹر�"
  //  1 : " Զ��ƿ���"
  uint8_t DCU_IDS_HighBeamAct : 1;           //      Bits= 1

  //  0 : " ˫���ر�"
  //  1 : " ˫������"
  uint8_t DCU_IDS_EmLightAct : 1;            //      Bits= 1

  // ����Ŀ�공�ٰ���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_CruiseResBtn : 1;          //      Bits= 1

  // ����Ŀ�공�ٰ���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_CruiseSetBtn : 1;          //      Bits= 1

  // ��ǰ�����־�����ư���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_CruiseDisPls : 1;          //      Bits= 1

  //  0 : " δ����"
  //  1 : " ��������"
  uint8_t DCU_IDS_HornStatus : 1;            //      Bits= 1

  //  0 : " ��ɲ���ɿ�"
  //  1 : " ��ɲ������"
  //  2 : " ��ɲ������"
  //  3 : " ��ɲ�ͷ���"
  uint8_t DCU_IDS_EPBStatus : 2;             //      Bits= 2

  //  0 : " �����ƹر�"
  //  1 : " �����ƿ���"
  uint8_t DCU_IDS_RearFogLampAct : 1;        //      Bits= 1

  //  0 : " ǰ���ƹر�"
  //  1 : " ǰ���ƿ���"
  uint8_t DCU_IDS_FrontFogLampAct : 1;       //      Bits= 1

  // ���������ܰ���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_LKSBtnInfo : 1;            //      Bits= 1

  //  0 : "����"
  //  1 : "�ر�"
  uint8_t DCU_IDS_BuzzerCmd : 1;             //      Bits= 1 Unit:'-'

  uint8_t DCU_IDS_GearStat_CTROLL : 4;       //      Bits= 4

  uint8_t DCU_IDS_GearStat_CHECKSUM;         //      Bits= 8

#else

  //  1 : " P��"
  //  2 : " R��"
  //  3 : " N��"
  //  4 : " D��"
  uint8_t DCU_IDS_GearAct;                   //      Bits= 4

  //  0 : " ����"
  //  1 : " P��"
  //  2 : " R��"
  //  3 : " N��"
  //  4 : " D��"
  //  5 : " ����"
  uint8_t DCU_IDS_GearCmd;                   //      Bits= 4

  //  0 : " ������ת���"
  //  1 : " ��ת�����˸"
  //  2 : " ��ת�����˸"
  //  3 : " ˫��"
  uint8_t DCU_IDS_TurnLightCmd;              //      Bits= 2

  //  0 : " ������ת���"
  //  1 : " ��ת�����˸"
  //  2 : " ��ת�����˸"
  //  3 : " ����"
  uint8_t DCU_IDS_TurnLightAct;              //      Bits= 2

  // ����ACC���ܰ���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_CruiseOnBtn;               //      Bits= 1

  // �ر�ACC���ܰ���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_CruiseCnclBtn;             //      Bits= 1

  //  0 : " ����ƹر�"
  //  1 : " ����ƿ���"
  uint8_t DCU_IDS_LowBeamAct;                //      Bits= 1

  //  0 : " Զ��ƹر�"
  //  1 : " Զ��ƿ���"
  uint8_t DCU_IDS_HighBeamAct;               //      Bits= 1

  //  0 : " ˫���ر�"
  //  1 : " ˫������"
  uint8_t DCU_IDS_EmLightAct;                //      Bits= 1

  // ����Ŀ�공�ٰ���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_CruiseResBtn;              //      Bits= 1

  // ����Ŀ�공�ٰ���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_CruiseSetBtn;              //      Bits= 1

  // ��ǰ�����־�����ư���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_CruiseDisPls;              //      Bits= 1

  //  0 : " δ����"
  //  1 : " ��������"
  uint8_t DCU_IDS_HornStatus;                //      Bits= 1

  //  0 : " ��ɲ���ɿ�"
  //  1 : " ��ɲ������"
  //  2 : " ��ɲ������"
  //  3 : " ��ɲ�ͷ���"
  uint8_t DCU_IDS_EPBStatus;                 //      Bits= 2

  //  0 : " �����ƹر�"
  //  1 : " �����ƿ���"
  uint8_t DCU_IDS_RearFogLampAct;            //      Bits= 1

  //  0 : " ǰ���ƹر�"
  //  1 : " ǰ���ƿ���"
  uint8_t DCU_IDS_FrontFogLampAct;           //      Bits= 1

  // ���������ܰ���
  //  0 : " ��ťδ����"
  //  1 : " ��ť����"
  uint8_t DCU_IDS_LKSBtnInfo;                //      Bits= 1

  //  0 : "����"
  //  1 : "�ر�"
  uint8_t DCU_IDS_BuzzerCmd;                 //      Bits= 1 Unit:'-'

  uint8_t DCU_IDS_GearStat_CTROLL;           //      Bits= 4

  uint8_t DCU_IDS_GearStat_CHECKSUM;         //      Bits= 8

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} DCU_IDS_MSG204_t;

// def @DCU_IDS_MSG205 CAN Message (517  0x205)
#define DCU_IDS_MSG205_IDE (0U)
#define DCU_IDS_MSG205_DLC (8U)
#define DCU_IDS_MSG205_CANID (0x205)
#define DCU_IDS_MSG205_CYC (20U)
// signal: @DCU_IDS_WheelSpdFL_ro
#define TESTDB_DCU_IDS_WheelSpdFL_ro_CovFactor (0.01)
#define TESTDB_DCU_IDS_WheelSpdFL_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define TESTDB_DCU_IDS_WheelSpdFL_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @DCU_IDS_WheelSpdFR_ro
#define TESTDB_DCU_IDS_WheelSpdFR_ro_CovFactor (0.01)
#define TESTDB_DCU_IDS_WheelSpdFR_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define TESTDB_DCU_IDS_WheelSpdFR_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @DCU_IDS_WheelSpdRL_ro
#define TESTDB_DCU_IDS_WheelSpdRL_ro_CovFactor (0.01)
#define TESTDB_DCU_IDS_WheelSpdRL_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define TESTDB_DCU_IDS_WheelSpdRL_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @DCU_IDS_WheelSpdRR_ro
#define TESTDB_DCU_IDS_WheelSpdRR_ro_CovFactor (0.01)
#define TESTDB_DCU_IDS_WheelSpdRR_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define TESTDB_DCU_IDS_WheelSpdRR_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  uint16_t DCU_IDS_WheelSpdFL_ro;            //      Bits=16 Factor= 0.01            Unit:'km/h'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_WheelSpdFL_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_WheelSpdFR_ro;            //      Bits=16 Factor= 0.01            Unit:'km/h'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_WheelSpdFR_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_WheelSpdRL_ro;            //      Bits=16 Factor= 0.01            Unit:'km/h'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_WheelSpdRL_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_WheelSpdRR_ro;            //      Bits=16 Factor= 0.01            Unit:'km/h'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_WheelSpdRR_phys;
#endif // TESTDB_USE_SIGFLOAT

#else

  uint16_t DCU_IDS_WheelSpdFL_ro;            //      Bits=16 Factor= 0.01            Unit:'km/h'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_WheelSpdFL_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_WheelSpdFR_ro;            //      Bits=16 Factor= 0.01            Unit:'km/h'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_WheelSpdFR_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_WheelSpdRL_ro;            //      Bits=16 Factor= 0.01            Unit:'km/h'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_WheelSpdRL_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_WheelSpdRR_ro;            //      Bits=16 Factor= 0.01            Unit:'km/h'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_WheelSpdRR_phys;
#endif // TESTDB_USE_SIGFLOAT

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} DCU_IDS_MSG205_t;

// def @DCU_IDS_MSG206 CAN Message (518  0x206)
#define DCU_IDS_MSG206_IDE (0U)
#define DCU_IDS_MSG206_DLC (8U)
#define DCU_IDS_MSG206_CANID (0x206)
#define DCU_IDS_MSG206_CYC (20U)
// signal: @DCU_IDS_VehSpd_ro
#define TESTDB_DCU_IDS_VehSpd_ro_CovFactor (0.01)
#define TESTDB_DCU_IDS_VehSpd_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define TESTDB_DCU_IDS_VehSpd_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @DCU_IDS_VehAccX_ro
#define TESTDB_DCU_IDS_VehAccX_ro_CovFactor (0.01)
#define TESTDB_DCU_IDS_VehAccX_ro_toS(x) ( (uint16_t) (((x) - (-20.0)) / (0.01)) )
#define TESTDB_DCU_IDS_VehAccX_ro_fromS(x) ( (((x) * (0.01)) + (-20.0)) )
// signal: @DCU_IDS_VehAccY_ro
#define TESTDB_DCU_IDS_VehAccY_ro_CovFactor (0.01)
#define TESTDB_DCU_IDS_VehAccY_ro_toS(x) ( (uint16_t) (((x) - (-300.0)) / (0.01)) )
#define TESTDB_DCU_IDS_VehAccY_ro_fromS(x) ( (((x) * (0.01)) + (-300.0)) )
// signal: @DCU_IDS_VehYawRate_ro
#define TESTDB_DCU_IDS_VehYawRate_ro_CovFactor (0.0006)
#define TESTDB_DCU_IDS_VehYawRate_ro_toS(x) ( (uint16_t) (((x) - (-20.0)) / (0.0006)) )
#define TESTDB_DCU_IDS_VehYawRate_ro_fromS(x) ( (((x) * (0.0006)) + (-20.0)) )

typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  uint16_t DCU_IDS_VehSpd_ro;                //      Bits=16 Factor= 0.01            Unit:'km/h'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_VehSpd_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_VehAccX_ro;               //      Bits=16 Offset= -20.0              Factor= 0.01            Unit:'m/s^2'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_VehAccX_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_VehAccY_ro;               //      Bits=16 Offset= -300.0             Factor= 0.01            Unit:'m/s^2'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_VehAccY_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_VehYawRate_ro;            //      Bits=16 Offset= -20.0              Factor= 0.0006          Unit:'rad/s'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_VehYawRate_phys;
#endif // TESTDB_USE_SIGFLOAT

#else

  uint16_t DCU_IDS_VehSpd_ro;                //      Bits=16 Factor= 0.01            Unit:'km/h'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_VehSpd_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_VehAccX_ro;               //      Bits=16 Offset= -20.0              Factor= 0.01            Unit:'m/s^2'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_VehAccX_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_VehAccY_ro;               //      Bits=16 Offset= -300.0             Factor= 0.01            Unit:'m/s^2'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_VehAccY_phys;
#endif // TESTDB_USE_SIGFLOAT

  uint16_t DCU_IDS_VehYawRate_ro;            //      Bits=16 Offset= -20.0              Factor= 0.0006          Unit:'rad/s'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_VehYawRate_phys;
#endif // TESTDB_USE_SIGFLOAT

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} DCU_IDS_MSG206_t;

// def @DCU_IDS_MSG207 CAN Message (519  0x207)
#define DCU_IDS_MSG207_IDE (0U)
#define DCU_IDS_MSG207_DLC (8U)
#define DCU_IDS_MSG207_CANID (0x207)
#define DCU_IDS_MSG207_CYC (100U)

// Value tables for @DCU_IDS_DoorFL signal

#ifndef DCU_IDS_DoorFL_DCU_IDS_MSG207__
#define DCU_IDS_DoorFL_DCU_IDS_MSG207__ (0)
#endif

#ifndef DCU_IDS_DoorFL_DCU_IDS_MSG207__
#define DCU_IDS_DoorFL_DCU_IDS_MSG207__ (1)
#endif


// Value tables for @DCU_IDS_DoorFR signal

#ifndef DCU_IDS_DoorFR_DCU_IDS_MSG207__
#define DCU_IDS_DoorFR_DCU_IDS_MSG207__ (0)
#endif

#ifndef DCU_IDS_DoorFR_DCU_IDS_MSG207__
#define DCU_IDS_DoorFR_DCU_IDS_MSG207__ (1)
#endif


// Value tables for @DCU_IDS_DoorRL signal

#ifndef DCU_IDS_DoorRL_DCU_IDS_MSG207__
#define DCU_IDS_DoorRL_DCU_IDS_MSG207__ (0)
#endif

#ifndef DCU_IDS_DoorRL_DCU_IDS_MSG207__
#define DCU_IDS_DoorRL_DCU_IDS_MSG207__ (1)
#endif


// Value tables for @DCU_IDS_DoorRR signal

#ifndef DCU_IDS_DoorRR_DCU_IDS_MSG207__
#define DCU_IDS_DoorRR_DCU_IDS_MSG207__ (0)
#endif

#ifndef DCU_IDS_DoorRR_DCU_IDS_MSG207__
#define DCU_IDS_DoorRR_DCU_IDS_MSG207__ (1)
#endif


// Value tables for @DCU_IDS_DriverSeatBeltSt signal

#ifndef DCU_IDS_DriverSeatBeltSt_DCU_IDS_MSG207__
#define DCU_IDS_DriverSeatBeltSt_DCU_IDS_MSG207__ (0)
#endif

#ifndef DCU_IDS_DriverSeatBeltSt_DCU_IDS_MSG207__
#define DCU_IDS_DriverSeatBeltSt_DCU_IDS_MSG207__ (1)
#endif

#ifndef DCU_IDS_DriverSeatBeltSt_DCU_IDS_MSG207__
#define DCU_IDS_DriverSeatBeltSt_DCU_IDS_MSG207__ (2)
#endif

#ifndef DCU_IDS_DriverSeatBeltSt_DCU_IDS_MSG207__
#define DCU_IDS_DriverSeatBeltSt_DCU_IDS_MSG207__ (3)
#endif


// Value tables for @DCU_IDS_KeySt signal

#ifndef DCU_IDS_KeySt_DCU_IDS_MSG207__OFF
#define DCU_IDS_KeySt_DCU_IDS_MSG207__OFF (0)
#endif

#ifndef DCU_IDS_KeySt_DCU_IDS_MSG207__ACC
#define DCU_IDS_KeySt_DCU_IDS_MSG207__ACC (1)
#endif

#ifndef DCU_IDS_KeySt_DCU_IDS_MSG207__ON
#define DCU_IDS_KeySt_DCU_IDS_MSG207__ON (2)
#endif

#ifndef DCU_IDS_KeySt_DCU_IDS_MSG207__CRANK
#define DCU_IDS_KeySt_DCU_IDS_MSG207__CRANK (3)
#endif


typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  //  0 : " �ر�"
  //  1 : " ��"
  uint8_t DCU_IDS_DoorFL : 1;                //      Bits= 1

  //  0 : " �ر�"
  //  1 : " ��"
  uint8_t DCU_IDS_DoorFR : 1;                //      Bits= 1

  //  0 : " �ر�"
  //  1 : " ��"
  uint8_t DCU_IDS_DoorRL : 1;                //      Bits= 1

  //  0 : " �ر�"
  //  1 : " ��"
  uint8_t DCU_IDS_DoorRR : 1;                //      Bits= 1

  //  0 : " ��ȫ������"
  //  1 : " ��ȫ��δ��"
  //  2 : " Ԥ��"
  //  3 : " Ԥ��"
  uint8_t DCU_IDS_DriverSeatBeltSt : 2;      //      Bits= 2

  //  0 : " OFF"
  //  1 : " ACC"
  //  2 : " ON"
  //  3 : " CRANK"
  uint8_t DCU_IDS_KeySt : 2;                 //      Bits= 2

#else

  //  0 : " �ر�"
  //  1 : " ��"
  uint8_t DCU_IDS_DoorFL;                    //      Bits= 1

  //  0 : " �ر�"
  //  1 : " ��"
  uint8_t DCU_IDS_DoorFR;                    //      Bits= 1

  //  0 : " �ر�"
  //  1 : " ��"
  uint8_t DCU_IDS_DoorRL;                    //      Bits= 1

  //  0 : " �ر�"
  //  1 : " ��"
  uint8_t DCU_IDS_DoorRR;                    //      Bits= 1

  //  0 : " ��ȫ������"
  //  1 : " ��ȫ��δ��"
  //  2 : " Ԥ��"
  //  3 : " Ԥ��"
  uint8_t DCU_IDS_DriverSeatBeltSt;          //      Bits= 2

  //  0 : " OFF"
  //  1 : " ACC"
  //  2 : " ON"
  //  3 : " CRANK"
  uint8_t DCU_IDS_KeySt;                     //      Bits= 2

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} DCU_IDS_MSG207_t;

// def @DCU_IDS_MSG208 CAN Message (520  0x208)
#define DCU_IDS_MSG208_IDE (0U)
#define DCU_IDS_MSG208_DLC (8U)
#define DCU_IDS_MSG208_CANID (0x208)
#define DCU_IDS_MSG208_CYC (100U)

typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  // Distance from rear right middle sensor
  uint8_t DCU_IDS_RRMidDistance;             //      Bits= 8 Unit:'cm'

  // Distance from rear left middle sensor
  uint8_t DCU_IDS_RLMidDistance;             //      Bits= 8 Unit:'cm'

  // Distance from rear right sensor
  uint8_t DCU_IDS_RRDistance;                //      Bits= 8 Unit:'cm'

  // Distance from rear left sensor
  uint8_t DCU_IDS_RLDistance;                //      Bits= 8 Unit:'cm'

  // Distance from rear left side sensor
  uint8_t DCU_IDS_RSLSideDistance;           //      Bits= 8 Unit:'cm'

  // Distance from rear right side sensor
  uint8_t DCU_IDS_RSRSideDistance;           //      Bits= 8 Unit:'cm'

#else

  // Distance from rear right middle sensor
  uint8_t DCU_IDS_RRMidDistance;             //      Bits= 8 Unit:'cm'

  // Distance from rear left middle sensor
  uint8_t DCU_IDS_RLMidDistance;             //      Bits= 8 Unit:'cm'

  // Distance from rear right sensor
  uint8_t DCU_IDS_RRDistance;                //      Bits= 8 Unit:'cm'

  // Distance from rear left sensor
  uint8_t DCU_IDS_RLDistance;                //      Bits= 8 Unit:'cm'

  // Distance from rear left side sensor
  uint8_t DCU_IDS_RSLSideDistance;           //      Bits= 8 Unit:'cm'

  // Distance from rear right side sensor
  uint8_t DCU_IDS_RSRSideDistance;           //      Bits= 8 Unit:'cm'

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} DCU_IDS_MSG208_t;

// def @DCU_IDS_MSG209 CAN Message (521  0x209)
#define DCU_IDS_MSG209_IDE (0U)
#define DCU_IDS_MSG209_DLC (8U)
#define DCU_IDS_MSG209_CANID (0x209)
#define DCU_IDS_MSG209_CYC (100U)

typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  // Distance from front right sensor
  uint8_t DCU_IDS_FRDistance;                //      Bits= 8 Unit:'cm'

  // Distance from front left sensor
  uint8_t DCU_IDS_FLDistance;                //      Bits= 8 Unit:'cm'

  // Distance from front right middle sensor
  uint8_t DCU_IDS_FRMidDistance;             //      Bits= 8 Unit:'cm'

  // Distance from front left middle sensor
  uint8_t DCU_IDS_FLMidDistance;             //      Bits= 8 Unit:'cm'

  // Distance from front right side sensor
  uint8_t DCU_IDS_FSRSideDistance;           //      Bits= 8 Unit:'cm'

  // Distance from front left side sensor
  uint8_t DCU_IDS_FSLSideDistance;           //      Bits= 8 Unit:'cm'

#else

  // Distance from front right sensor
  uint8_t DCU_IDS_FRDistance;                //      Bits= 8 Unit:'cm'

  // Distance from front left sensor
  uint8_t DCU_IDS_FLDistance;                //      Bits= 8 Unit:'cm'

  // Distance from front right middle sensor
  uint8_t DCU_IDS_FRMidDistance;             //      Bits= 8 Unit:'cm'

  // Distance from front left middle sensor
  uint8_t DCU_IDS_FLMidDistance;             //      Bits= 8 Unit:'cm'

  // Distance from front right side sensor
  uint8_t DCU_IDS_FSRSideDistance;           //      Bits= 8 Unit:'cm'

  // Distance from front left side sensor
  uint8_t DCU_IDS_FSLSideDistance;           //      Bits= 8 Unit:'cm'

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} DCU_IDS_MSG209_t;

// This is a message for not used signals, created by Vector CANdb++ DBC OLE DB Provider.
// def @DCU_IDS_MSG20C CAN Message (524  0x20c)
#define DCU_IDS_MSG20C_IDE (0U)
#define DCU_IDS_MSG20C_DLC (8U)
#define DCU_IDS_MSG20C_CANID (0x20c)
#define DCU_IDS_MSG20C_CYC (100U)
// signal: @DCU_IDS_CorrectedExterTemp_ro
#define TESTDB_DCU_IDS_CorrectedExterTemp_ro_CovFactor (0.5)
#define TESTDB_DCU_IDS_CorrectedExterTemp_ro_toS(x) ( (uint8_t) (((x) - (-40.0)) / (0.5)) )
#define TESTDB_DCU_IDS_CorrectedExterTemp_ro_fromS(x) ( (((x) * (0.5)) + (-40.0)) )

// Value tables for @DCU_IDS_CorrectedExterTempVD signal

#ifndef DCU_IDS_CorrectedExterTempVD_DCU_IDS_MSG20C__Valid0
#define DCU_IDS_CorrectedExterTempVD_DCU_IDS_MSG20C__Valid0 (1)
#endif


typedef struct
{
#ifdef TESTDB_USE_BITS_SIGNAL

  uint8_t DCU_IDS_CorrectedExterTemp_ro;       //      Bits= 8 Offset= -40.0              Factor= 0.5             Unit:'��C'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_CorrectedExterTemp_phys;
#endif // TESTDB_USE_SIGFLOAT

  // External temperature validity1: Valid
  // 0: Not valid
  //  1 : " Valid0"
  uint8_t DCU_IDS_CorrectedExterTempVD : 1;    //      Bits= 1

  // Total odometer�ܼ����
  uint32_t DCU_IDS_TotalOdometer;              //      Bits=20 Unit:'km'

#else

  uint8_t DCU_IDS_CorrectedExterTemp_ro;       //      Bits= 8 Offset= -40.0              Factor= 0.5             Unit:'��C'

#ifdef TESTDB_USE_SIGFLOAT
  sigfloat_t DCU_IDS_CorrectedExterTemp_phys;
#endif // TESTDB_USE_SIGFLOAT

  // External temperature validity1: Valid
  // 0: Not valid
  //  1 : " Valid0"
  uint8_t DCU_IDS_CorrectedExterTempVD;        //      Bits= 1

  // Total odometer�ܼ����
  uint32_t DCU_IDS_TotalOdometer;              //      Bits=20 Unit:'km'

#endif // TESTDB_USE_BITS_SIGNAL

#ifdef TESTDB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // TESTDB_USE_DIAG_MONITORS

} DCU_IDS_MSG20C_t;

// Function signatures

uint32_t Unpack_IDS_DCU_MSG100_testdb(IDS_DCU_MSG100_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_IDS_DCU_MSG100_testdb(IDS_DCU_MSG100_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_IDS_DCU_MSG100_testdb(IDS_DCU_MSG100_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_IDS_DCU_MSG101_testdb(IDS_DCU_MSG101_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_IDS_DCU_MSG101_testdb(IDS_DCU_MSG101_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_IDS_DCU_MSG101_testdb(IDS_DCU_MSG101_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_IDS_DCU_MSG102_testdb(IDS_DCU_MSG102_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_IDS_DCU_MSG102_testdb(IDS_DCU_MSG102_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_IDS_DCU_MSG102_testdb(IDS_DCU_MSG102_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_IDS_DCU_MSG103_testdb(IDS_DCU_MSG103_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_IDS_DCU_MSG103_testdb(IDS_DCU_MSG103_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_IDS_DCU_MSG103_testdb(IDS_DCU_MSG103_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_IDS_DCU_MSG104_testdb(IDS_DCU_MSG104_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_IDS_DCU_MSG104_testdb(IDS_DCU_MSG104_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_IDS_DCU_MSG104_testdb(IDS_DCU_MSG104_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG200_testdb(DCU_IDS_MSG200_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_DCU_IDS_MSG200_testdb(DCU_IDS_MSG200_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DCU_IDS_MSG200_testdb(DCU_IDS_MSG200_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG201_testdb(DCU_IDS_MSG201_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_DCU_IDS_MSG201_testdb(DCU_IDS_MSG201_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DCU_IDS_MSG201_testdb(DCU_IDS_MSG201_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG202_testdb(DCU_IDS_MSG202_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_DCU_IDS_MSG202_testdb(DCU_IDS_MSG202_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DCU_IDS_MSG202_testdb(DCU_IDS_MSG202_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG203_testdb(DCU_IDS_MSG203_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_DCU_IDS_MSG203_testdb(DCU_IDS_MSG203_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DCU_IDS_MSG203_testdb(DCU_IDS_MSG203_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG204_testdb(DCU_IDS_MSG204_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_DCU_IDS_MSG204_testdb(DCU_IDS_MSG204_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DCU_IDS_MSG204_testdb(DCU_IDS_MSG204_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG205_testdb(DCU_IDS_MSG205_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_DCU_IDS_MSG205_testdb(DCU_IDS_MSG205_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DCU_IDS_MSG205_testdb(DCU_IDS_MSG205_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG206_testdb(DCU_IDS_MSG206_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_DCU_IDS_MSG206_testdb(DCU_IDS_MSG206_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DCU_IDS_MSG206_testdb(DCU_IDS_MSG206_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG207_testdb(DCU_IDS_MSG207_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_DCU_IDS_MSG207_testdb(DCU_IDS_MSG207_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DCU_IDS_MSG207_testdb(DCU_IDS_MSG207_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG208_testdb(DCU_IDS_MSG208_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_DCU_IDS_MSG208_testdb(DCU_IDS_MSG208_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DCU_IDS_MSG208_testdb(DCU_IDS_MSG208_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG209_testdb(DCU_IDS_MSG209_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_DCU_IDS_MSG209_testdb(DCU_IDS_MSG209_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DCU_IDS_MSG209_testdb(DCU_IDS_MSG209_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG20C_testdb(DCU_IDS_MSG20C_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef TESTDB_USE_CANSTRUCT
uint32_t Pack_DCU_IDS_MSG20C_testdb(DCU_IDS_MSG20C_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DCU_IDS_MSG20C_testdb(DCU_IDS_MSG20C_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // TESTDB_USE_CANSTRUCT

#ifdef __cplusplus
}
#endif
