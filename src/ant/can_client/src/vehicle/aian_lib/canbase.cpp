
#include "vehicle/aian_lib/canbase.hpp"


// DBC file version
#if (VER_TESTDB_MAJ != (0U)) || (VER_TESTDB_MIN != (0U))
#error The TESTDB dbc source files have different versions
#endif

#ifdef TESTDB_USE_DIAG_MONITORS
// Function prototypes to be called each time CAN frame is unpacked
// FMon function may detect RC, CRC or DLC violation
#include <testdb-fmon.h>

#endif // TESTDB_USE_DIAG_MONITORS

// This macro guard for the case when you need to enable
// using diag monitors but there is no necessity in proper
// SysTick provider. For providing one you need define macro
// before this line - in dbccodeconf.h

#ifndef GetSystemTick
#define GetSystemTick() (0u)
#endif

// This macro guard is for the case when you want to build
// app with enabled optoin auto CSM, but don't yet have
// proper getframehash implementation

#ifndef GetFrameHash
#define GetFrameHash(a,b,c,d,e) (0u)
#endif

// This function performs extension of sign for the signals
// which have non-aligned to power of 2 bit's width.
// The types 'bitext_t' and 'ubitext_t' define maximal bit width which
// can be correctly handled. You need to select type which can contain
// n+1 bits where n is the largest signed signal width. For example if
// the most wide signed signal has a width of 31 bits you need to set
// bitext_t as int32_t and ubitext_t as uint32_t
// Defined these typedefs in @dbccodeconf.h or locally in 'dbcdrvname'-config.h
static bitext_t __ext_sig__(ubitext_t val, uint8_t bits)
{
  ubitext_t const m = 1u << (bits - 1);
  return (val ^ m) - m;
}

uint32_t Unpack_IDS_DCU_MSG100_testdb(IDS_DCU_MSG100_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->IDS_DCU_AutoCtrlEna = (_d[0] & (0x01U));
  _m->IDS_ACU_TakeOverMode = ((_d[0] >> 6) & (0x01U));
  _m->IDS_ACU_VehManulToAutoMode = ((_d[0] >> 7) & (0x01U));
  _m->IDS_DCU_ModeCtrl_CTROLL = (_d[6] & (0x0FU));
  _m->IDS_DCU_ModeCtrl_CHECKSUM = (_d[7] & (0xFFU));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < IDS_DCU_MSG100_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_IDS_DCU_MSG100_testdb(&_m->mon1, IDS_DCU_MSG100_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return IDS_DCU_MSG100_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_IDS_DCU_MSG100_testdb(IDS_DCU_MSG100_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < IDS_DCU_MSG100_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->IDS_DCU_AutoCtrlEna & (0x01U)) | ((_m->IDS_ACU_TakeOverMode & (0x01U)) << 6) | ((_m->IDS_ACU_VehManulToAutoMode & (0x01U)) << 7);
  cframe->Data[6] |= (_m->IDS_DCU_ModeCtrl_CTROLL & (0x0FU));
  cframe->Data[7] |= (_m->IDS_DCU_ModeCtrl_CHECKSUM & (0xFFU));

  cframe->MsgId = IDS_DCU_MSG100_CANID;
  cframe->DLC = IDS_DCU_MSG100_DLC;
  cframe->IDE = IDS_DCU_MSG100_IDE;
  return IDS_DCU_MSG100_CANID;
}

#else

uint32_t Pack_IDS_DCU_MSG100_testdb(IDS_DCU_MSG100_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < IDS_DCU_MSG100_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->IDS_DCU_AutoCtrlEna & (0x01U)) | ((_m->IDS_ACU_TakeOverMode & (0x01U)) << 6) | ((_m->IDS_ACU_VehManulToAutoMode & (0x01U)) << 7);
  _d[6] |= (_m->IDS_DCU_ModeCtrl_CTROLL & (0x0FU));
  _d[7] |= (_m->IDS_DCU_ModeCtrl_CHECKSUM & (0xFFU));

  *_len = IDS_DCU_MSG100_DLC;
  *_ide = IDS_DCU_MSG100_IDE;
  return IDS_DCU_MSG100_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_IDS_DCU_MSG101_testdb(IDS_DCU_MSG101_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->IDS_DCU_AccPedCmd_ro = ((_d[0] & (0x03U)) << 8) | (_d[1] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_AccPedCmd_phys = (sigfloat_t)(TESTDB_IDS_DCU_AccPedCmd_ro_fromS(_m->IDS_DCU_AccPedCmd_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->IDS_DCU_AccCtrMode = ((_d[0] >> 5) & (0x01U));
  _m->IDS_DCU_AccOverRideDisable = ((_d[0] >> 6) & (0x01U));
  _m->IDS_DCU_AccCtrlEna = ((_d[0] >> 7) & (0x01U));
  _m->IDS_DCU_AccPedInv_ro = ((_d[2] & (0x03U)) << 8) | (_d[3] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_AccPedInv_phys = (sigfloat_t)(TESTDB_IDS_DCU_AccPedInv_ro_fromS(_m->IDS_DCU_AccPedInv_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->IDS_DCU_WheelTorqCmd_ro = ((_d[4] & (0x3FU)) << 8) | (_d[5] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_WheelTorqCmd_phys = TESTDB_IDS_DCU_WheelTorqCmd_ro_fromS(_m->IDS_DCU_WheelTorqCmd_ro);
#endif // TESTDB_USE_SIGFLOAT

  _m->IDS_DCU_WheelTorqInv_ro = ((_d[6] & (0x3FU)) << 8) | (_d[7] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_WheelTorqInv_phys = TESTDB_IDS_DCU_WheelTorqInv_ro_fromS(_m->IDS_DCU_WheelTorqInv_ro);
#endif // TESTDB_USE_SIGFLOAT

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < IDS_DCU_MSG101_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_IDS_DCU_MSG101_testdb(&_m->mon1, IDS_DCU_MSG101_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return IDS_DCU_MSG101_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_IDS_DCU_MSG101_testdb(IDS_DCU_MSG101_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < IDS_DCU_MSG101_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_AccPedCmd_ro = TESTDB_IDS_DCU_AccPedCmd_ro_toS(_m->IDS_DCU_AccPedCmd_phys);
  _m->IDS_DCU_AccPedInv_ro = TESTDB_IDS_DCU_AccPedInv_ro_toS(_m->IDS_DCU_AccPedInv_phys);
  _m->IDS_DCU_WheelTorqCmd_ro = TESTDB_IDS_DCU_WheelTorqCmd_ro_toS(_m->IDS_DCU_WheelTorqCmd_phys);
  _m->IDS_DCU_WheelTorqInv_ro = TESTDB_IDS_DCU_WheelTorqInv_ro_toS(_m->IDS_DCU_WheelTorqInv_phys);
#endif // TESTDB_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->IDS_DCU_AccPedCmd_ro >> 8) & (0x03U)) | ((_m->IDS_DCU_AccCtrMode & (0x01U)) << 5) | ((_m->IDS_DCU_AccOverRideDisable & (0x01U)) << 6) | ((_m->IDS_DCU_AccCtrlEna & (0x01U)) << 7);
  cframe->Data[1] |= (_m->IDS_DCU_AccPedCmd_ro & (0xFFU));
  cframe->Data[2] |= ((_m->IDS_DCU_AccPedInv_ro >> 8) & (0x03U));
  cframe->Data[3] |= (_m->IDS_DCU_AccPedInv_ro & (0xFFU));
  cframe->Data[4] |= ((_m->IDS_DCU_WheelTorqCmd_ro >> 8) & (0x3FU));
  cframe->Data[5] |= (_m->IDS_DCU_WheelTorqCmd_ro & (0xFFU));
  cframe->Data[6] |= ((_m->IDS_DCU_WheelTorqInv_ro >> 8) & (0x3FU));
  cframe->Data[7] |= (_m->IDS_DCU_WheelTorqInv_ro & (0xFFU));

  cframe->MsgId = IDS_DCU_MSG101_CANID;
  cframe->DLC = IDS_DCU_MSG101_DLC;
  cframe->IDE = IDS_DCU_MSG101_IDE;
  return IDS_DCU_MSG101_CANID;
}

#else

uint32_t Pack_IDS_DCU_MSG101_testdb(IDS_DCU_MSG101_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < IDS_DCU_MSG101_DLC) && (i < 8); _d[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_AccPedCmd_ro = TESTDB_IDS_DCU_AccPedCmd_ro_toS(_m->IDS_DCU_AccPedCmd_phys);
  _m->IDS_DCU_AccPedInv_ro = TESTDB_IDS_DCU_AccPedInv_ro_toS(_m->IDS_DCU_AccPedInv_phys);
  _m->IDS_DCU_WheelTorqCmd_ro = TESTDB_IDS_DCU_WheelTorqCmd_ro_toS(_m->IDS_DCU_WheelTorqCmd_phys);
  _m->IDS_DCU_WheelTorqInv_ro = TESTDB_IDS_DCU_WheelTorqInv_ro_toS(_m->IDS_DCU_WheelTorqInv_phys);
#endif // TESTDB_USE_SIGFLOAT

  _d[0] |= ((_m->IDS_DCU_AccPedCmd_ro >> 8) & (0x03U)) | ((_m->IDS_DCU_AccCtrMode & (0x01U)) << 5) | ((_m->IDS_DCU_AccOverRideDisable & (0x01U)) << 6) | ((_m->IDS_DCU_AccCtrlEna & (0x01U)) << 7);
  _d[1] |= (_m->IDS_DCU_AccPedCmd_ro & (0xFFU));
  _d[2] |= ((_m->IDS_DCU_AccPedInv_ro >> 8) & (0x03U));
  _d[3] |= (_m->IDS_DCU_AccPedInv_ro & (0xFFU));
  _d[4] |= ((_m->IDS_DCU_WheelTorqCmd_ro >> 8) & (0x3FU));
  _d[5] |= (_m->IDS_DCU_WheelTorqCmd_ro & (0xFFU));
  _d[6] |= ((_m->IDS_DCU_WheelTorqInv_ro >> 8) & (0x3FU));
  _d[7] |= (_m->IDS_DCU_WheelTorqInv_ro & (0xFFU));

  *_len = IDS_DCU_MSG101_DLC;
  *_ide = IDS_DCU_MSG101_IDE;
  return IDS_DCU_MSG101_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_IDS_DCU_MSG102_testdb(IDS_DCU_MSG102_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->IDS_DCU_BrkPedCmd_ro = ((_d[0] & (0x03U)) << 8) | (_d[1] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_BrkPedCmd_phys = (sigfloat_t)(TESTDB_IDS_DCU_BrkPedCmd_ro_fromS(_m->IDS_DCU_BrkPedCmd_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->IDS_DCU_BrkCtrMode = ((_d[0] >> 5) & (0x01U));
  _m->IDS_DCU_BrkOverRideDisable = ((_d[0] >> 6) & (0x01U));
  _m->IDS_DCU_BrkCtrlEna = ((_d[0] >> 7) & (0x01U));
  _m->IDS_DCU_BrkPedInv_ro = ((_d[2] & (0x03U)) << 8) | (_d[3] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_BrkPedInv_phys = (sigfloat_t)(TESTDB_IDS_DCU_BrkPedInv_ro_fromS(_m->IDS_DCU_BrkPedInv_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->IDS_DCU_TargDecelCmd_ro = (_d[4] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_TargDecelCmd_phys = (sigfloat_t)(TESTDB_IDS_DCU_TargDecelCmd_ro_fromS(_m->IDS_DCU_TargDecelCmd_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->IDS_DCU_TargDecelInv_ro = (_d[5] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_TargDecelInv_phys = (sigfloat_t)(TESTDB_IDS_DCU_TargDecelInv_ro_fromS(_m->IDS_DCU_TargDecelInv_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->IDS_DCU_BrkCtrl_CTROLL = (_d[6] & (0x0FU));
  _m->IDS_DCU_BrkCtrl_CHECKSUM = (_d[7] & (0xFFU));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < IDS_DCU_MSG102_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_IDS_DCU_MSG102_testdb(&_m->mon1, IDS_DCU_MSG102_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return IDS_DCU_MSG102_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_IDS_DCU_MSG102_testdb(IDS_DCU_MSG102_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < IDS_DCU_MSG102_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_BrkPedCmd_ro = TESTDB_IDS_DCU_BrkPedCmd_ro_toS(_m->IDS_DCU_BrkPedCmd_phys);
  _m->IDS_DCU_BrkPedInv_ro = TESTDB_IDS_DCU_BrkPedInv_ro_toS(_m->IDS_DCU_BrkPedInv_phys);
  _m->IDS_DCU_TargDecelCmd_ro = TESTDB_IDS_DCU_TargDecelCmd_ro_toS(_m->IDS_DCU_TargDecelCmd_phys);
  _m->IDS_DCU_TargDecelInv_ro = TESTDB_IDS_DCU_TargDecelInv_ro_toS(_m->IDS_DCU_TargDecelInv_phys);
#endif // TESTDB_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->IDS_DCU_BrkPedCmd_ro >> 8) & (0x03U)) | ((_m->IDS_DCU_BrkCtrMode & (0x01U)) << 5) | ((_m->IDS_DCU_BrkOverRideDisable & (0x01U)) << 6) | ((_m->IDS_DCU_BrkCtrlEna & (0x01U)) << 7);
  cframe->Data[1] |= (_m->IDS_DCU_BrkPedCmd_ro & (0xFFU));
  cframe->Data[2] |= ((_m->IDS_DCU_BrkPedInv_ro >> 8) & (0x03U));
  cframe->Data[3] |= (_m->IDS_DCU_BrkPedInv_ro & (0xFFU));
  cframe->Data[4] |= (_m->IDS_DCU_TargDecelCmd_ro & (0xFFU));
  cframe->Data[5] |= (_m->IDS_DCU_TargDecelInv_ro & (0xFFU));
  cframe->Data[6] |= (_m->IDS_DCU_BrkCtrl_CTROLL & (0x0FU));
  cframe->Data[7] |= (_m->IDS_DCU_BrkCtrl_CHECKSUM & (0xFFU));

  cframe->MsgId = IDS_DCU_MSG102_CANID;
  cframe->DLC = IDS_DCU_MSG102_DLC;
  cframe->IDE = IDS_DCU_MSG102_IDE;
  return IDS_DCU_MSG102_CANID;
}

#else

uint32_t Pack_IDS_DCU_MSG102_testdb(IDS_DCU_MSG102_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < IDS_DCU_MSG102_DLC) && (i < 8); _d[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_BrkPedCmd_ro = TESTDB_IDS_DCU_BrkPedCmd_ro_toS(_m->IDS_DCU_BrkPedCmd_phys);
  _m->IDS_DCU_BrkPedInv_ro = TESTDB_IDS_DCU_BrkPedInv_ro_toS(_m->IDS_DCU_BrkPedInv_phys);
  _m->IDS_DCU_TargDecelCmd_ro = TESTDB_IDS_DCU_TargDecelCmd_ro_toS(_m->IDS_DCU_TargDecelCmd_phys);
  _m->IDS_DCU_TargDecelInv_ro = TESTDB_IDS_DCU_TargDecelInv_ro_toS(_m->IDS_DCU_TargDecelInv_phys);
#endif // TESTDB_USE_SIGFLOAT

  _d[0] |= ((_m->IDS_DCU_BrkPedCmd_ro >> 8) & (0x03U)) | ((_m->IDS_DCU_BrkCtrMode & (0x01U)) << 5) | ((_m->IDS_DCU_BrkOverRideDisable & (0x01U)) << 6) | ((_m->IDS_DCU_BrkCtrlEna & (0x01U)) << 7);
  _d[1] |= (_m->IDS_DCU_BrkPedCmd_ro & (0xFFU));
  _d[2] |= ((_m->IDS_DCU_BrkPedInv_ro >> 8) & (0x03U));
  _d[3] |= (_m->IDS_DCU_BrkPedInv_ro & (0xFFU));
  _d[4] |= (_m->IDS_DCU_TargDecelCmd_ro & (0xFFU));
  _d[5] |= (_m->IDS_DCU_TargDecelInv_ro & (0xFFU));
  _d[6] |= (_m->IDS_DCU_BrkCtrl_CTROLL & (0x0FU));
  _d[7] |= (_m->IDS_DCU_BrkCtrl_CHECKSUM & (0xFFU));

  *_len = IDS_DCU_MSG102_DLC;
  *_ide = IDS_DCU_MSG102_IDE;
  return IDS_DCU_MSG102_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_IDS_DCU_MSG103_testdb(IDS_DCU_MSG103_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->IDS_DCU_StrAngCmd_ro = ((_d[0] & (0x3FU)) << 8) | (_d[1] & (0xFFU)) - 8000;
#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_StrAngCmd_phys = (sigfloat_t)(TESTDB_IDS_DCU_StrAngCmd_ro_fromS(_m->IDS_DCU_StrAngCmd_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->IDS_DCU_StrCtrMode = ((_d[0] >> 6) & (0x01U));
  _m->IDS_DCU_StrCtrlEna = ((_d[0] >> 7) & (0x01U));
  _m->IDS_DCU_StrAngLimit_ro = ((_d[2] & (0x3FU)) << 8) | (_d[3] & (0xFFU)) - 8000;
#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_StrAngLimit_phys = (sigfloat_t)(TESTDB_IDS_DCU_StrAngLimit_ro_fromS(_m->IDS_DCU_StrAngLimit_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->IDS_DCU_StrAngRateLimit = ((_d[4] & (0x0FU)) << 8) | (_d[5] & (0xFFU));
  _m->IDS_DCU_StrCtrl_CTROLL = ((_d[4] >> 4) & (0x0FU));
  _m->IDS_DCU_StrTorq_ro = ((_d[6] & (0xFFU)) << 3) | ((_d[7] >> 5) & (0x07U));
#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_StrTorq_phys = (sigfloat_t)(TESTDB_IDS_DCU_StrTorq_ro_fromS(_m->IDS_DCU_StrTorq_ro));
#endif // TESTDB_USE_SIGFLOAT

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < IDS_DCU_MSG103_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_IDS_DCU_MSG103_testdb(&_m->mon1, IDS_DCU_MSG103_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return IDS_DCU_MSG103_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_IDS_DCU_MSG103_testdb(IDS_DCU_MSG103_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < IDS_DCU_MSG103_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_StrAngCmd_ro = TESTDB_IDS_DCU_StrAngCmd_ro_toS(_m->IDS_DCU_StrAngCmd_phys);
  _m->IDS_DCU_StrAngLimit_ro = TESTDB_IDS_DCU_StrAngLimit_ro_toS(_m->IDS_DCU_StrAngLimit_phys);
  _m->IDS_DCU_StrTorq_ro = TESTDB_IDS_DCU_StrTorq_ro_toS(_m->IDS_DCU_StrTorq_phys);
#endif // TESTDB_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->IDS_DCU_StrAngCmd_ro >> 8) & (0x3FU)) | ((_m->IDS_DCU_StrCtrMode & (0x01U)) << 6) | ((_m->IDS_DCU_StrCtrlEna & (0x01U)) << 7);
  cframe->Data[1] |= (_m->IDS_DCU_StrAngCmd_ro & (0xFFU));
  cframe->Data[2] |= ((_m->IDS_DCU_StrAngLimit_ro >> 8) & (0x3FU));
  cframe->Data[3] |= (_m->IDS_DCU_StrAngLimit_ro & (0xFFU));
  cframe->Data[4] |= ((_m->IDS_DCU_StrAngRateLimit >> 8) & (0x0FU)) | ((_m->IDS_DCU_StrCtrl_CTROLL & (0x0FU)) << 4);
  cframe->Data[5] |= (_m->IDS_DCU_StrAngRateLimit & (0xFFU));
  cframe->Data[6] |= ((_m->IDS_DCU_StrTorq_ro >> 3) & (0xFFU));
  cframe->Data[7] |= ((_m->IDS_DCU_StrTorq_ro & (0x07U)) << 5);

  cframe->MsgId = IDS_DCU_MSG103_CANID;
  cframe->DLC = IDS_DCU_MSG103_DLC;
  cframe->IDE = IDS_DCU_MSG103_IDE;
  return IDS_DCU_MSG103_CANID;
}

#else

uint32_t Pack_IDS_DCU_MSG103_testdb(IDS_DCU_MSG103_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < IDS_DCU_MSG103_DLC) && (i < 8); _d[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->IDS_DCU_StrAngCmd_ro = TESTDB_IDS_DCU_StrAngCmd_ro_toS(_m->IDS_DCU_StrAngCmd_phys);
  _m->IDS_DCU_StrAngLimit_ro = TESTDB_IDS_DCU_StrAngLimit_ro_toS(_m->IDS_DCU_StrAngLimit_phys);
  _m->IDS_DCU_StrTorq_ro = TESTDB_IDS_DCU_StrTorq_ro_toS(_m->IDS_DCU_StrTorq_phys);
#endif // TESTDB_USE_SIGFLOAT

  _d[0] |= (((_m->IDS_DCU_StrAngCmd_ro+8000) >> 8) & (0x3FU)) | ((_m->IDS_DCU_StrCtrMode & (0x01U)) << 6) | ((_m->IDS_DCU_StrCtrlEna & (0x01U)) << 7);
  _d[1] |= ((_m->IDS_DCU_StrAngCmd_ro+8000) & (0xFFU));
  _d[2] |= (((_m->IDS_DCU_StrAngLimit_ro+8000) >> 8) & (0x3FU));
  _d[3] |= ((_m->IDS_DCU_StrAngLimit_ro+8000) & (0xFFU));
  _d[4] |= ((_m->IDS_DCU_StrAngRateLimit >> 8) & (0x0FU)) | ((_m->IDS_DCU_StrCtrl_CTROLL & (0x0FU)) << 4);
  _d[5] |= (_m->IDS_DCU_StrAngRateLimit & (0xFFU));
  _d[6] |= ((_m->IDS_DCU_StrTorq_ro >> 3) & (0xFFU));
  _d[7] |= ((_m->IDS_DCU_StrTorq_ro & (0x07U)) << 5);

  *_len = IDS_DCU_MSG103_DLC;
  *_ide = IDS_DCU_MSG103_IDE;
  return IDS_DCU_MSG103_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_IDS_DCU_MSG104_testdb(IDS_DCU_MSG104_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->IDS_DCU_GearCmd = (_d[0] & (0x0FU));
  _m->IDS_DCU_BuzzerCmd = ((_d[0] >> 4) & (0x01U));
  _m->IDS_DCU_GearCtrlEna = ((_d[0] >> 7) & (0x01U));
  _m->IDS_DCU_TurnLightCmd = (_d[1] & (0x03U));
  _m->IDS_DCU_BeamCmd = ((_d[1] >> 2) & (0x03U));
  _m->IDS_DCU_EPBCmd = ((_d[1] >> 4) & (0x03U));
  _m->IDS_DCU_HornCmd = ((_d[1] >> 6) & (0x01U));
  _m->IDS_ACU_GearManulToAutoMode = ((_d[1] >> 7) & (0x01U));
  _m->IDS_DCU_Gear_CTROLL = (_d[6] & (0x0FU));
  _m->IDS_DCU_WiperCmd = ((_d[6] >> 4) & (0x03U));
  _m->IDS_DCU_Gear_CHECKSUM = (_d[7] & (0xFFU));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < IDS_DCU_MSG104_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_IDS_DCU_MSG104_testdb(&_m->mon1, IDS_DCU_MSG104_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return IDS_DCU_MSG104_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_IDS_DCU_MSG104_testdb(IDS_DCU_MSG104_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < IDS_DCU_MSG104_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->IDS_DCU_GearCmd & (0x0FU)) | ((_m->IDS_DCU_BuzzerCmd & (0x01U)) << 4) | ((_m->IDS_DCU_GearCtrlEna & (0x01U)) << 7);
  cframe->Data[1] |= (_m->IDS_DCU_TurnLightCmd & (0x03U)) | ((_m->IDS_DCU_BeamCmd & (0x03U)) << 2) | ((_m->IDS_DCU_EPBCmd & (0x03U)) << 4) | ((_m->IDS_DCU_HornCmd & (0x01U)) << 6) | ((_m->IDS_ACU_GearManulToAutoMode & (0x01U)) << 7);
  cframe->Data[6] |= (_m->IDS_DCU_Gear_CTROLL & (0x0FU)) | ((_m->IDS_DCU_WiperCmd & (0x03U)) << 4);
  cframe->Data[7] |= (_m->IDS_DCU_Gear_CHECKSUM & (0xFFU));

  cframe->MsgId = IDS_DCU_MSG104_CANID;
  cframe->DLC = IDS_DCU_MSG104_DLC;
  cframe->IDE = IDS_DCU_MSG104_IDE;
  return IDS_DCU_MSG104_CANID;
}

#else

uint32_t Pack_IDS_DCU_MSG104_testdb(IDS_DCU_MSG104_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < IDS_DCU_MSG104_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->IDS_DCU_GearCmd & (0x0FU)) | ((_m->IDS_DCU_BuzzerCmd & (0x01U)) << 4) | ((_m->IDS_DCU_GearCtrlEna & (0x01U)) << 7);
  _d[1] |= (_m->IDS_DCU_TurnLightCmd & (0x03U)) | ((_m->IDS_DCU_BeamCmd & (0x03U)) << 2) | ((_m->IDS_DCU_EPBCmd & (0x03U)) << 4) | ((_m->IDS_DCU_HornCmd & (0x01U)) << 6) | ((_m->IDS_ACU_GearManulToAutoMode & (0x01U)) << 7);
  _d[6] |= (_m->IDS_DCU_Gear_CTROLL & (0x0FU)) | ((_m->IDS_DCU_WiperCmd & (0x03U)) << 4);
  _d[7] |= (_m->IDS_DCU_Gear_CHECKSUM & (0xFFU));

  *_len = IDS_DCU_MSG104_DLC;
  *_ide = IDS_DCU_MSG104_IDE;
  return IDS_DCU_MSG104_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG200_testdb(DCU_IDS_MSG200_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DCU_IDS_AutoCtrlStat = (_d[0] & (0x03U));
  _m->DCU_IDS_FRWheelRotatedDirection = ((_d[0] >> 2) & (0x01U));
  _m->DCU_IDS_FLWheelRotatedDirection = ((_d[0] >> 3) & (0x01U));
  _m->DCU_IDS_RRWheelRotatedDirection = ((_d[0] >> 4) & (0x01U));
  _m->DCU_IDS_RLWheelRotatedDirection = ((_d[0] >> 5) & (0x01U));
  _m->DCU_IDS_WhlPlsFr = (_d[1] & (0xFFU));
  _m->DCU_IDS_WhlPlsFl = (_d[2] & (0xFFU));
  _m->DCU_IDS_WhlPlsRl = (_d[3] & (0xFFU));
  _m->DCU_IDS_WhlPlsRr = (_d[4] & (0xFFU));
  _m->DCU_IDS_ModeStat_CTROLL = (_d[6] & (0x0FU));
  _m->DCU_IDS_bBrkTakeover = ((_d[6] >> 4) & (0x01U));
  _m->DCU_IDS_bAccTakeover = ((_d[6] >> 5) & (0x01U));
  _m->DCU_IDS_bStrTakeover = ((_d[6] >> 6) & (0x01U));
  _m->DCU_IDS_bOtherTakeover = ((_d[6] >> 7) & (0x01U));
  _m->DCU_IDS_ModeStat_CHECKSUM = (_d[7] & (0xFFU));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DCU_IDS_MSG200_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DCU_IDS_MSG200_testdb(&_m->mon1, DCU_IDS_MSG200_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return DCU_IDS_MSG200_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_DCU_IDS_MSG200_testdb(DCU_IDS_MSG200_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG200_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->DCU_IDS_AutoCtrlStat & (0x03U)) | ((_m->DCU_IDS_FRWheelRotatedDirection & (0x01U)) << 2) | ((_m->DCU_IDS_FLWheelRotatedDirection & (0x01U)) << 3) | ((_m->DCU_IDS_RRWheelRotatedDirection & (0x01U)) << 4) | ((_m->DCU_IDS_RLWheelRotatedDirection & (0x01U)) << 5);
  cframe->Data[1] |= (_m->DCU_IDS_WhlPlsFr & (0xFFU));
  cframe->Data[2] |= (_m->DCU_IDS_WhlPlsFl & (0xFFU));
  cframe->Data[3] |= (_m->DCU_IDS_WhlPlsRl & (0xFFU));
  cframe->Data[4] |= (_m->DCU_IDS_WhlPlsRr & (0xFFU));
  cframe->Data[6] |= (_m->DCU_IDS_ModeStat_CTROLL & (0x0FU)) | ((_m->DCU_IDS_bBrkTakeover & (0x01U)) << 4) | ((_m->DCU_IDS_bAccTakeover & (0x01U)) << 5) | ((_m->DCU_IDS_bStrTakeover & (0x01U)) << 6) | ((_m->DCU_IDS_bOtherTakeover & (0x01U)) << 7);
  cframe->Data[7] |= (_m->DCU_IDS_ModeStat_CHECKSUM & (0xFFU));

  cframe->MsgId = DCU_IDS_MSG200_CANID;
  cframe->DLC = DCU_IDS_MSG200_DLC;
  cframe->IDE = DCU_IDS_MSG200_IDE;
  return DCU_IDS_MSG200_CANID;
}

#else

uint32_t Pack_DCU_IDS_MSG200_testdb(DCU_IDS_MSG200_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG200_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->DCU_IDS_AutoCtrlStat & (0x03U)) | ((_m->DCU_IDS_FRWheelRotatedDirection & (0x01U)) << 2) | ((_m->DCU_IDS_FLWheelRotatedDirection & (0x01U)) << 3) | ((_m->DCU_IDS_RRWheelRotatedDirection & (0x01U)) << 4) | ((_m->DCU_IDS_RLWheelRotatedDirection & (0x01U)) << 5);
  _d[1] |= (_m->DCU_IDS_WhlPlsFr & (0xFFU));
  _d[2] |= (_m->DCU_IDS_WhlPlsFl & (0xFFU));
  _d[3] |= (_m->DCU_IDS_WhlPlsRl & (0xFFU));
  _d[4] |= (_m->DCU_IDS_WhlPlsRr & (0xFFU));
  _d[6] |= (_m->DCU_IDS_ModeStat_CTROLL & (0x0FU)) | ((_m->DCU_IDS_bBrkTakeover & (0x01U)) << 4) | ((_m->DCU_IDS_bAccTakeover & (0x01U)) << 5) | ((_m->DCU_IDS_bStrTakeover & (0x01U)) << 6) | ((_m->DCU_IDS_bOtherTakeover & (0x01U)) << 7);
  _d[7] |= (_m->DCU_IDS_ModeStat_CHECKSUM & (0xFFU));

  *_len = DCU_IDS_MSG200_DLC;
  *_ide = DCU_IDS_MSG200_IDE;
  return DCU_IDS_MSG200_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG201_testdb(DCU_IDS_MSG201_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DCU_IDS_AccPedAct_ro = ((_d[0] & (0x03U)) << 8) | (_d[1] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_AccPedAct_phys = (sigfloat_t)(TESTDB_DCU_IDS_AccPedAct_ro_fromS(_m->DCU_IDS_AccPedAct_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_AccCtrlModeSt = ((_d[0] >> 2) & (0x03U));
  _m->DCU_IDS_AccCtrlStat = ((_d[0] >> 6) & (0x03U));
  _m->DCU_IDS_AccPedCmd_ro = ((_d[2] & (0x03U)) << 8) | (_d[3] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_AccPedCmd_phys = (sigfloat_t)(TESTDB_DCU_IDS_AccPedCmd_ro_fromS(_m->DCU_IDS_AccPedCmd_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_AccPedExe_ro = ((_d[4] & (0x03U)) << 8) | (_d[5] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_AccPedExe_phys = (sigfloat_t)(TESTDB_DCU_IDS_AccPedExe_ro_fromS(_m->DCU_IDS_AccPedExe_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_ActVehWheelTorq = ((_d[6] & (0x3FU)) << 8) | (_d[7] & (0xFFU));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DCU_IDS_MSG201_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DCU_IDS_MSG201_testdb(&_m->mon1, DCU_IDS_MSG201_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return DCU_IDS_MSG201_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_DCU_IDS_MSG201_testdb(DCU_IDS_MSG201_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG201_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_AccPedAct_ro = TESTDB_DCU_IDS_AccPedAct_ro_toS(_m->DCU_IDS_AccPedAct_phys);
  _m->DCU_IDS_AccPedCmd_ro = TESTDB_DCU_IDS_AccPedCmd_ro_toS(_m->DCU_IDS_AccPedCmd_phys);
  _m->DCU_IDS_AccPedExe_ro = TESTDB_DCU_IDS_AccPedExe_ro_toS(_m->DCU_IDS_AccPedExe_phys);
#endif // TESTDB_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->DCU_IDS_AccPedAct_ro >> 8) & (0x03U)) | ((_m->DCU_IDS_AccCtrlModeSt & (0x03U)) << 2) | ((_m->DCU_IDS_AccCtrlStat & (0x03U)) << 6);
  cframe->Data[1] |= (_m->DCU_IDS_AccPedAct_ro & (0xFFU));
  cframe->Data[2] |= ((_m->DCU_IDS_AccPedCmd_ro >> 8) & (0x03U));
  cframe->Data[3] |= (_m->DCU_IDS_AccPedCmd_ro & (0xFFU));
  cframe->Data[4] |= ((_m->DCU_IDS_AccPedExe_ro >> 8) & (0x03U));
  cframe->Data[5] |= (_m->DCU_IDS_AccPedExe_ro & (0xFFU));
  cframe->Data[6] |= ((_m->DCU_IDS_ActVehWheelTorq >> 8) & (0x3FU));
  cframe->Data[7] |= (_m->DCU_IDS_ActVehWheelTorq & (0xFFU));

  cframe->MsgId = DCU_IDS_MSG201_CANID;
  cframe->DLC = DCU_IDS_MSG201_DLC;
  cframe->IDE = DCU_IDS_MSG201_IDE;
  return DCU_IDS_MSG201_CANID;
}

#else

uint32_t Pack_DCU_IDS_MSG201_testdb(DCU_IDS_MSG201_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG201_DLC) && (i < 8); _d[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_AccPedAct_ro = TESTDB_DCU_IDS_AccPedAct_ro_toS(_m->DCU_IDS_AccPedAct_phys);
  _m->DCU_IDS_AccPedCmd_ro = TESTDB_DCU_IDS_AccPedCmd_ro_toS(_m->DCU_IDS_AccPedCmd_phys);
  _m->DCU_IDS_AccPedExe_ro = TESTDB_DCU_IDS_AccPedExe_ro_toS(_m->DCU_IDS_AccPedExe_phys);
#endif // TESTDB_USE_SIGFLOAT

  _d[0] |= ((_m->DCU_IDS_AccPedAct_ro >> 8) & (0x03U)) | ((_m->DCU_IDS_AccCtrlModeSt & (0x03U)) << 2) | ((_m->DCU_IDS_AccCtrlStat & (0x03U)) << 6);
  _d[1] |= (_m->DCU_IDS_AccPedAct_ro & (0xFFU));
  _d[2] |= ((_m->DCU_IDS_AccPedCmd_ro >> 8) & (0x03U));
  _d[3] |= (_m->DCU_IDS_AccPedCmd_ro & (0xFFU));
  _d[4] |= ((_m->DCU_IDS_AccPedExe_ro >> 8) & (0x03U));
  _d[5] |= (_m->DCU_IDS_AccPedExe_ro & (0xFFU));
  _d[6] |= ((_m->DCU_IDS_ActVehWheelTorq >> 8) & (0x3FU));
  _d[7] |= (_m->DCU_IDS_ActVehWheelTorq & (0xFFU));

  *_len = DCU_IDS_MSG201_DLC;
  *_ide = DCU_IDS_MSG201_IDE;
  return DCU_IDS_MSG201_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG202_testdb(DCU_IDS_MSG202_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DCU_IDS_BrkPedAct_ro = ((_d[0] & (0x03U)) << 8) | (_d[1] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_BrkPedAct_phys = (sigfloat_t)(TESTDB_DCU_IDS_BrkPedAct_ro_fromS(_m->DCU_IDS_BrkPedAct_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_BrkCtrlModeSt = ((_d[0] >> 2) & (0x03U));
  _m->DCU_IDS_BrkCtrlStat = ((_d[0] >> 6) & (0x03U));
  _m->DCU_IDS_BrkPedCmd_ro = ((_d[2] & (0x03U)) << 8) | (_d[3] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_BrkPedCmd_phys = (sigfloat_t)(TESTDB_DCU_IDS_BrkPedCmd_ro_fromS(_m->DCU_IDS_BrkPedCmd_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_BrkPedExe_ro = ((_d[4] & (0x03U)) << 8) | (_d[5] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_BrkPedExe_phys = (sigfloat_t)(TESTDB_DCU_IDS_BrkPedExe_ro_fromS(_m->DCU_IDS_BrkPedExe_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_BrkStat_CTROLL = (_d[6] & (0x0FU));
  _m->DCU_IDS_BrkStat_CHECKSUM = (_d[7] & (0xFFU));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DCU_IDS_MSG202_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DCU_IDS_MSG202_testdb(&_m->mon1, DCU_IDS_MSG202_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return DCU_IDS_MSG202_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_DCU_IDS_MSG202_testdb(DCU_IDS_MSG202_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG202_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_BrkPedAct_ro = TESTDB_DCU_IDS_BrkPedAct_ro_toS(_m->DCU_IDS_BrkPedAct_phys);
  _m->DCU_IDS_BrkPedCmd_ro = TESTDB_DCU_IDS_BrkPedCmd_ro_toS(_m->DCU_IDS_BrkPedCmd_phys);
  _m->DCU_IDS_BrkPedExe_ro = TESTDB_DCU_IDS_BrkPedExe_ro_toS(_m->DCU_IDS_BrkPedExe_phys);
#endif // TESTDB_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->DCU_IDS_BrkPedAct_ro >> 8) & (0x03U)) | ((_m->DCU_IDS_BrkCtrlModeSt & (0x03U)) << 2) | ((_m->DCU_IDS_BrkCtrlStat & (0x03U)) << 6);
  cframe->Data[1] |= (_m->DCU_IDS_BrkPedAct_ro & (0xFFU));
  cframe->Data[2] |= ((_m->DCU_IDS_BrkPedCmd_ro >> 8) & (0x03U));
  cframe->Data[3] |= (_m->DCU_IDS_BrkPedCmd_ro & (0xFFU));
  cframe->Data[4] |= ((_m->DCU_IDS_BrkPedExe_ro >> 8) & (0x03U));
  cframe->Data[5] |= (_m->DCU_IDS_BrkPedExe_ro & (0xFFU));
  cframe->Data[6] |= (_m->DCU_IDS_BrkStat_CTROLL & (0x0FU));
  cframe->Data[7] |= (_m->DCU_IDS_BrkStat_CHECKSUM & (0xFFU));

  cframe->MsgId = DCU_IDS_MSG202_CANID;
  cframe->DLC = DCU_IDS_MSG202_DLC;
  cframe->IDE = DCU_IDS_MSG202_IDE;
  return DCU_IDS_MSG202_CANID;
}

#else

uint32_t Pack_DCU_IDS_MSG202_testdb(DCU_IDS_MSG202_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG202_DLC) && (i < 8); _d[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_BrkPedAct_ro = TESTDB_DCU_IDS_BrkPedAct_ro_toS(_m->DCU_IDS_BrkPedAct_phys);
  _m->DCU_IDS_BrkPedCmd_ro = TESTDB_DCU_IDS_BrkPedCmd_ro_toS(_m->DCU_IDS_BrkPedCmd_phys);
  _m->DCU_IDS_BrkPedExe_ro = TESTDB_DCU_IDS_BrkPedExe_ro_toS(_m->DCU_IDS_BrkPedExe_phys);
#endif // TESTDB_USE_SIGFLOAT

  _d[0] |= ((_m->DCU_IDS_BrkPedAct_ro >> 8) & (0x03U)) | ((_m->DCU_IDS_BrkCtrlModeSt & (0x03U)) << 2) | ((_m->DCU_IDS_BrkCtrlStat & (0x03U)) << 6);
  _d[1] |= (_m->DCU_IDS_BrkPedAct_ro & (0xFFU));
  _d[2] |= ((_m->DCU_IDS_BrkPedCmd_ro >> 8) & (0x03U));
  _d[3] |= (_m->DCU_IDS_BrkPedCmd_ro & (0xFFU));
  _d[4] |= ((_m->DCU_IDS_BrkPedExe_ro >> 8) & (0x03U));
  _d[5] |= (_m->DCU_IDS_BrkPedExe_ro & (0xFFU));
  _d[6] |= (_m->DCU_IDS_BrkStat_CTROLL & (0x0FU));
  _d[7] |= (_m->DCU_IDS_BrkStat_CHECKSUM & (0xFFU));

  *_len = DCU_IDS_MSG202_DLC;
  *_ide = DCU_IDS_MSG202_IDE;
  return DCU_IDS_MSG202_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG203_testdb(DCU_IDS_MSG203_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DCU_IDS_StrAngAct_ro = ((_d[0] & (0x3FU)) << 8) | (_d[1] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_StrAngAct_phys = (sigfloat_t)(TESTDB_DCU_IDS_StrAngAct_ro_fromS(_m->DCU_IDS_StrAngAct_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_StrCtrlStat = ((_d[0] >> 6) & (0x03U));
  _m->DCU_IDS_StrAngCmd_ro = ((_d[2] & (0x3FU)) << 8) | (_d[3] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_StrAngCmd_phys = (sigfloat_t)(TESTDB_DCU_IDS_StrAngCmd_ro_fromS(_m->DCU_IDS_StrAngCmd_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_bEpsCtrFailed = ((_d[2] >> 6) & (0x01U));
  _m->DCU_IDS_StrTrqAct_ro = (_d[4] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_StrTrqAct_phys = (sigfloat_t)(TESTDB_DCU_IDS_StrTrqAct_ro_fromS(_m->DCU_IDS_StrTrqAct_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_SteeringAngleSpd_ro = (_d[5] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_SteeringAngleSpd_phys = TESTDB_DCU_IDS_SteeringAngleSpd_ro_fromS(_m->DCU_IDS_SteeringAngleSpd_ro);
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_StrStat_CTROLL = (_d[6] & (0x0FU));
  _m->DCU_IDS_StrStat_CHECKSUM = (_d[7] & (0xFFU));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DCU_IDS_MSG203_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DCU_IDS_MSG203_testdb(&_m->mon1, DCU_IDS_MSG203_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return DCU_IDS_MSG203_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_DCU_IDS_MSG203_testdb(DCU_IDS_MSG203_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG203_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_StrAngAct_ro = TESTDB_DCU_IDS_StrAngAct_ro_toS(_m->DCU_IDS_StrAngAct_phys);
  _m->DCU_IDS_StrAngCmd_ro = TESTDB_DCU_IDS_StrAngCmd_ro_toS(_m->DCU_IDS_StrAngCmd_phys);
  _m->DCU_IDS_StrTrqAct_ro = TESTDB_DCU_IDS_StrTrqAct_ro_toS(_m->DCU_IDS_StrTrqAct_phys);
  _m->DCU_IDS_SteeringAngleSpd_ro = TESTDB_DCU_IDS_SteeringAngleSpd_ro_toS(_m->DCU_IDS_SteeringAngleSpd_phys);
#endif // TESTDB_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->DCU_IDS_StrAngAct_ro >> 8) & (0x3FU)) | ((_m->DCU_IDS_StrCtrlStat & (0x03U)) << 6);
  cframe->Data[1] |= (_m->DCU_IDS_StrAngAct_ro & (0xFFU));
  cframe->Data[2] |= ((_m->DCU_IDS_StrAngCmd_ro >> 8) & (0x3FU)) | ((_m->DCU_IDS_bEpsCtrFailed & (0x01U)) << 6);
  cframe->Data[3] |= (_m->DCU_IDS_StrAngCmd_ro & (0xFFU));
  cframe->Data[4] |= (_m->DCU_IDS_StrTrqAct_ro & (0xFFU));
  cframe->Data[5] |= (_m->DCU_IDS_SteeringAngleSpd_ro & (0xFFU));
  cframe->Data[6] |= (_m->DCU_IDS_StrStat_CTROLL & (0x0FU));
  cframe->Data[7] |= (_m->DCU_IDS_StrStat_CHECKSUM & (0xFFU));

  cframe->MsgId = DCU_IDS_MSG203_CANID;
  cframe->DLC = DCU_IDS_MSG203_DLC;
  cframe->IDE = DCU_IDS_MSG203_IDE;
  return DCU_IDS_MSG203_CANID;
}

#else

uint32_t Pack_DCU_IDS_MSG203_testdb(DCU_IDS_MSG203_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG203_DLC) && (i < 8); _d[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_StrAngAct_ro = TESTDB_DCU_IDS_StrAngAct_ro_toS(_m->DCU_IDS_StrAngAct_phys);
  _m->DCU_IDS_StrAngCmd_ro = TESTDB_DCU_IDS_StrAngCmd_ro_toS(_m->DCU_IDS_StrAngCmd_phys);
  _m->DCU_IDS_StrTrqAct_ro = TESTDB_DCU_IDS_StrTrqAct_ro_toS(_m->DCU_IDS_StrTrqAct_phys);
  _m->DCU_IDS_SteeringAngleSpd_ro = TESTDB_DCU_IDS_SteeringAngleSpd_ro_toS(_m->DCU_IDS_SteeringAngleSpd_phys);
#endif // TESTDB_USE_SIGFLOAT

  _d[0] |= ((_m->DCU_IDS_StrAngAct_ro >> 8) & (0x3FU)) | ((_m->DCU_IDS_StrCtrlStat & (0x03U)) << 6);
  _d[1] |= (_m->DCU_IDS_StrAngAct_ro & (0xFFU));
  _d[2] |= ((_m->DCU_IDS_StrAngCmd_ro >> 8) & (0x3FU)) | ((_m->DCU_IDS_bEpsCtrFailed & (0x01U)) << 6);
  _d[3] |= (_m->DCU_IDS_StrAngCmd_ro & (0xFFU));
  _d[4] |= (_m->DCU_IDS_StrTrqAct_ro & (0xFFU));
  _d[5] |= (_m->DCU_IDS_SteeringAngleSpd_ro & (0xFFU));
  _d[6] |= (_m->DCU_IDS_StrStat_CTROLL & (0x0FU));
  _d[7] |= (_m->DCU_IDS_StrStat_CHECKSUM & (0xFFU));

  *_len = DCU_IDS_MSG203_DLC;
  *_ide = DCU_IDS_MSG203_IDE;
  return DCU_IDS_MSG203_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG204_testdb(DCU_IDS_MSG204_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DCU_IDS_GearAct = (_d[0] & (0x0FU));
  _m->DCU_IDS_GearCmd = ((_d[0] >> 4) & (0x0FU));
  _m->DCU_IDS_TurnLightCmd = (_d[1] & (0x03U));
  _m->DCU_IDS_TurnLightAct = ((_d[1] >> 2) & (0x03U));
  _m->DCU_IDS_CruiseOnBtn = ((_d[1] >> 4) & (0x01U));
  _m->DCU_IDS_CruiseCnclBtn = ((_d[1] >> 5) & (0x01U));
  _m->DCU_IDS_LowBeamAct = ((_d[1] >> 6) & (0x01U));
  _m->DCU_IDS_HighBeamAct = ((_d[1] >> 7) & (0x01U));
  _m->DCU_IDS_EmLightAct = (_d[2] & (0x01U));
  _m->DCU_IDS_CruiseResBtn = ((_d[2] >> 1) & (0x01U));
  _m->DCU_IDS_CruiseSetBtn = ((_d[2] >> 2) & (0x01U));
  _m->DCU_IDS_CruiseDisPls = ((_d[2] >> 3) & (0x01U));
  _m->DCU_IDS_HornStatus = ((_d[2] >> 5) & (0x01U));
  _m->DCU_IDS_EPBStatus = ((_d[2] >> 6) & (0x03U));
  _m->DCU_IDS_RearFogLampAct = (_d[3] & (0x01U));
  _m->DCU_IDS_FrontFogLampAct = ((_d[3] >> 1) & (0x01U));
  _m->DCU_IDS_LKSBtnInfo = ((_d[3] >> 2) & (0x01U));
  _m->DCU_IDS_BuzzerCmd = ((_d[3] >> 7) & (0x01U));
  _m->DCU_IDS_GearStat_CTROLL = (_d[6] & (0x0FU));
  _m->DCU_IDS_GearStat_CHECKSUM = (_d[7] & (0xFFU));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DCU_IDS_MSG204_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DCU_IDS_MSG204_testdb(&_m->mon1, DCU_IDS_MSG204_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return DCU_IDS_MSG204_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_DCU_IDS_MSG204_testdb(DCU_IDS_MSG204_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG204_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->DCU_IDS_GearAct & (0x0FU)) | ((_m->DCU_IDS_GearCmd & (0x0FU)) << 4);
  cframe->Data[1] |= (_m->DCU_IDS_TurnLightCmd & (0x03U)) | ((_m->DCU_IDS_TurnLightAct & (0x03U)) << 2) | ((_m->DCU_IDS_CruiseOnBtn & (0x01U)) << 4) | ((_m->DCU_IDS_CruiseCnclBtn & (0x01U)) << 5) | ((_m->DCU_IDS_LowBeamAct & (0x01U)) << 6) | ((_m->DCU_IDS_HighBeamAct & (0x01U)) << 7);
  cframe->Data[2] |= (_m->DCU_IDS_EmLightAct & (0x01U)) | ((_m->DCU_IDS_CruiseResBtn & (0x01U)) << 1) | ((_m->DCU_IDS_CruiseSetBtn & (0x01U)) << 2) | ((_m->DCU_IDS_CruiseDisPls & (0x01U)) << 3) | ((_m->DCU_IDS_HornStatus & (0x01U)) << 5) | ((_m->DCU_IDS_EPBStatus & (0x03U)) << 6);
  cframe->Data[3] |= (_m->DCU_IDS_RearFogLampAct & (0x01U)) | ((_m->DCU_IDS_FrontFogLampAct & (0x01U)) << 1) | ((_m->DCU_IDS_LKSBtnInfo & (0x01U)) << 2) | ((_m->DCU_IDS_BuzzerCmd & (0x01U)) << 7);
  cframe->Data[6] |= (_m->DCU_IDS_GearStat_CTROLL & (0x0FU));
  cframe->Data[7] |= (_m->DCU_IDS_GearStat_CHECKSUM & (0xFFU));

  cframe->MsgId = DCU_IDS_MSG204_CANID;
  cframe->DLC = DCU_IDS_MSG204_DLC;
  cframe->IDE = DCU_IDS_MSG204_IDE;
  return DCU_IDS_MSG204_CANID;
}

#else

uint32_t Pack_DCU_IDS_MSG204_testdb(DCU_IDS_MSG204_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG204_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->DCU_IDS_GearAct & (0x0FU)) | ((_m->DCU_IDS_GearCmd & (0x0FU)) << 4);
  _d[1] |= (_m->DCU_IDS_TurnLightCmd & (0x03U)) | ((_m->DCU_IDS_TurnLightAct & (0x03U)) << 2) | ((_m->DCU_IDS_CruiseOnBtn & (0x01U)) << 4) | ((_m->DCU_IDS_CruiseCnclBtn & (0x01U)) << 5) | ((_m->DCU_IDS_LowBeamAct & (0x01U)) << 6) | ((_m->DCU_IDS_HighBeamAct & (0x01U)) << 7);
  _d[2] |= (_m->DCU_IDS_EmLightAct & (0x01U)) | ((_m->DCU_IDS_CruiseResBtn & (0x01U)) << 1) | ((_m->DCU_IDS_CruiseSetBtn & (0x01U)) << 2) | ((_m->DCU_IDS_CruiseDisPls & (0x01U)) << 3) | ((_m->DCU_IDS_HornStatus & (0x01U)) << 5) | ((_m->DCU_IDS_EPBStatus & (0x03U)) << 6);
  _d[3] |= (_m->DCU_IDS_RearFogLampAct & (0x01U)) | ((_m->DCU_IDS_FrontFogLampAct & (0x01U)) << 1) | ((_m->DCU_IDS_LKSBtnInfo & (0x01U)) << 2) | ((_m->DCU_IDS_BuzzerCmd & (0x01U)) << 7);
  _d[6] |= (_m->DCU_IDS_GearStat_CTROLL & (0x0FU));
  _d[7] |= (_m->DCU_IDS_GearStat_CHECKSUM & (0xFFU));

  *_len = DCU_IDS_MSG204_DLC;
  *_ide = DCU_IDS_MSG204_IDE;
  return DCU_IDS_MSG204_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG205_testdb(DCU_IDS_MSG205_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DCU_IDS_WheelSpdFL_ro = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_WheelSpdFL_phys = (sigfloat_t)(TESTDB_DCU_IDS_WheelSpdFL_ro_fromS(_m->DCU_IDS_WheelSpdFL_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_WheelSpdFR_ro = ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_WheelSpdFR_phys = (sigfloat_t)(TESTDB_DCU_IDS_WheelSpdFR_ro_fromS(_m->DCU_IDS_WheelSpdFR_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_WheelSpdRL_ro = ((_d[4] & (0xFFU)) << 8) | (_d[5] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_WheelSpdRL_phys = (sigfloat_t)(TESTDB_DCU_IDS_WheelSpdRL_ro_fromS(_m->DCU_IDS_WheelSpdRL_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_WheelSpdRR_ro = ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_WheelSpdRR_phys = (sigfloat_t)(TESTDB_DCU_IDS_WheelSpdRR_ro_fromS(_m->DCU_IDS_WheelSpdRR_ro));
#endif // TESTDB_USE_SIGFLOAT

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DCU_IDS_MSG205_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DCU_IDS_MSG205_testdb(&_m->mon1, DCU_IDS_MSG205_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return DCU_IDS_MSG205_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_DCU_IDS_MSG205_testdb(DCU_IDS_MSG205_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG205_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_WheelSpdFL_ro = TESTDB_DCU_IDS_WheelSpdFL_ro_toS(_m->DCU_IDS_WheelSpdFL_phys);
  _m->DCU_IDS_WheelSpdFR_ro = TESTDB_DCU_IDS_WheelSpdFR_ro_toS(_m->DCU_IDS_WheelSpdFR_phys);
  _m->DCU_IDS_WheelSpdRL_ro = TESTDB_DCU_IDS_WheelSpdRL_ro_toS(_m->DCU_IDS_WheelSpdRL_phys);
  _m->DCU_IDS_WheelSpdRR_ro = TESTDB_DCU_IDS_WheelSpdRR_ro_toS(_m->DCU_IDS_WheelSpdRR_phys);
#endif // TESTDB_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->DCU_IDS_WheelSpdFL_ro >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->DCU_IDS_WheelSpdFL_ro & (0xFFU));
  cframe->Data[2] |= ((_m->DCU_IDS_WheelSpdFR_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->DCU_IDS_WheelSpdFR_ro & (0xFFU));
  cframe->Data[4] |= ((_m->DCU_IDS_WheelSpdRL_ro >> 8) & (0xFFU));
  cframe->Data[5] |= (_m->DCU_IDS_WheelSpdRL_ro & (0xFFU));
  cframe->Data[6] |= ((_m->DCU_IDS_WheelSpdRR_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->DCU_IDS_WheelSpdRR_ro & (0xFFU));

  cframe->MsgId = DCU_IDS_MSG205_CANID;
  cframe->DLC = DCU_IDS_MSG205_DLC;
  cframe->IDE = DCU_IDS_MSG205_IDE;
  return DCU_IDS_MSG205_CANID;
}

#else

uint32_t Pack_DCU_IDS_MSG205_testdb(DCU_IDS_MSG205_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG205_DLC) && (i < 8); _d[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_WheelSpdFL_ro = TESTDB_DCU_IDS_WheelSpdFL_ro_toS(_m->DCU_IDS_WheelSpdFL_phys);
  _m->DCU_IDS_WheelSpdFR_ro = TESTDB_DCU_IDS_WheelSpdFR_ro_toS(_m->DCU_IDS_WheelSpdFR_phys);
  _m->DCU_IDS_WheelSpdRL_ro = TESTDB_DCU_IDS_WheelSpdRL_ro_toS(_m->DCU_IDS_WheelSpdRL_phys);
  _m->DCU_IDS_WheelSpdRR_ro = TESTDB_DCU_IDS_WheelSpdRR_ro_toS(_m->DCU_IDS_WheelSpdRR_phys);
#endif // TESTDB_USE_SIGFLOAT

  _d[0] |= ((_m->DCU_IDS_WheelSpdFL_ro >> 8) & (0xFFU));
  _d[1] |= (_m->DCU_IDS_WheelSpdFL_ro & (0xFFU));
  _d[2] |= ((_m->DCU_IDS_WheelSpdFR_ro >> 8) & (0xFFU));
  _d[3] |= (_m->DCU_IDS_WheelSpdFR_ro & (0xFFU));
  _d[4] |= ((_m->DCU_IDS_WheelSpdRL_ro >> 8) & (0xFFU));
  _d[5] |= (_m->DCU_IDS_WheelSpdRL_ro & (0xFFU));
  _d[6] |= ((_m->DCU_IDS_WheelSpdRR_ro >> 8) & (0xFFU));
  _d[7] |= (_m->DCU_IDS_WheelSpdRR_ro & (0xFFU));

  *_len = DCU_IDS_MSG205_DLC;
  *_ide = DCU_IDS_MSG205_IDE;
  return DCU_IDS_MSG205_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG206_testdb(DCU_IDS_MSG206_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DCU_IDS_VehSpd_ro = ((_d[0] & (0xFFU)) << 8) | (_d[1] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_VehSpd_phys = (sigfloat_t)(TESTDB_DCU_IDS_VehSpd_ro_fromS(_m->DCU_IDS_VehSpd_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_VehAccX_ro = ((_d[2] & (0xFFU)) << 8) | (_d[3] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_VehAccX_phys = (sigfloat_t)(TESTDB_DCU_IDS_VehAccX_ro_fromS(_m->DCU_IDS_VehAccX_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_VehAccY_ro = ((_d[4] & (0xFFU)) << 8) | (_d[5] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_VehAccY_phys = (sigfloat_t)(TESTDB_DCU_IDS_VehAccY_ro_fromS(_m->DCU_IDS_VehAccY_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_VehYawRate_ro = ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_VehYawRate_phys = (sigfloat_t)(TESTDB_DCU_IDS_VehYawRate_ro_fromS(_m->DCU_IDS_VehYawRate_ro));
#endif // TESTDB_USE_SIGFLOAT

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DCU_IDS_MSG206_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DCU_IDS_MSG206_testdb(&_m->mon1, DCU_IDS_MSG206_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return DCU_IDS_MSG206_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_DCU_IDS_MSG206_testdb(DCU_IDS_MSG206_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG206_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_VehSpd_ro = TESTDB_DCU_IDS_VehSpd_ro_toS(_m->DCU_IDS_VehSpd_phys);
  _m->DCU_IDS_VehAccX_ro = TESTDB_DCU_IDS_VehAccX_ro_toS(_m->DCU_IDS_VehAccX_phys);
  _m->DCU_IDS_VehAccY_ro = TESTDB_DCU_IDS_VehAccY_ro_toS(_m->DCU_IDS_VehAccY_phys);
  _m->DCU_IDS_VehYawRate_ro = TESTDB_DCU_IDS_VehYawRate_ro_toS(_m->DCU_IDS_VehYawRate_phys);
#endif // TESTDB_USE_SIGFLOAT

  cframe->Data[0] |= ((_m->DCU_IDS_VehSpd_ro >> 8) & (0xFFU));
  cframe->Data[1] |= (_m->DCU_IDS_VehSpd_ro & (0xFFU));
  cframe->Data[2] |= ((_m->DCU_IDS_VehAccX_ro >> 8) & (0xFFU));
  cframe->Data[3] |= (_m->DCU_IDS_VehAccX_ro & (0xFFU));
  cframe->Data[4] |= ((_m->DCU_IDS_VehAccY_ro >> 8) & (0xFFU));
  cframe->Data[5] |= (_m->DCU_IDS_VehAccY_ro & (0xFFU));
  cframe->Data[6] |= ((_m->DCU_IDS_VehYawRate_ro >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->DCU_IDS_VehYawRate_ro & (0xFFU));

  cframe->MsgId = DCU_IDS_MSG206_CANID;
  cframe->DLC = DCU_IDS_MSG206_DLC;
  cframe->IDE = DCU_IDS_MSG206_IDE;
  return DCU_IDS_MSG206_CANID;
}

#else

uint32_t Pack_DCU_IDS_MSG206_testdb(DCU_IDS_MSG206_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG206_DLC) && (i < 8); _d[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_VehSpd_ro = TESTDB_DCU_IDS_VehSpd_ro_toS(_m->DCU_IDS_VehSpd_phys);
  _m->DCU_IDS_VehAccX_ro = TESTDB_DCU_IDS_VehAccX_ro_toS(_m->DCU_IDS_VehAccX_phys);
  _m->DCU_IDS_VehAccY_ro = TESTDB_DCU_IDS_VehAccY_ro_toS(_m->DCU_IDS_VehAccY_phys);
  _m->DCU_IDS_VehYawRate_ro = TESTDB_DCU_IDS_VehYawRate_ro_toS(_m->DCU_IDS_VehYawRate_phys);
#endif // TESTDB_USE_SIGFLOAT

  _d[0] |= ((_m->DCU_IDS_VehSpd_ro >> 8) & (0xFFU));
  _d[1] |= (_m->DCU_IDS_VehSpd_ro & (0xFFU));
  _d[2] |= ((_m->DCU_IDS_VehAccX_ro >> 8) & (0xFFU));
  _d[3] |= (_m->DCU_IDS_VehAccX_ro & (0xFFU));
  _d[4] |= ((_m->DCU_IDS_VehAccY_ro >> 8) & (0xFFU));
  _d[5] |= (_m->DCU_IDS_VehAccY_ro & (0xFFU));
  _d[6] |= ((_m->DCU_IDS_VehYawRate_ro >> 8) & (0xFFU));
  _d[7] |= (_m->DCU_IDS_VehYawRate_ro & (0xFFU));

  *_len = DCU_IDS_MSG206_DLC;
  *_ide = DCU_IDS_MSG206_IDE;
  return DCU_IDS_MSG206_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG207_testdb(DCU_IDS_MSG207_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DCU_IDS_DoorFL = (_d[0] & (0x01U));
  _m->DCU_IDS_DoorFR = ((_d[0] >> 1) & (0x01U));
  _m->DCU_IDS_DoorRL = ((_d[0] >> 2) & (0x01U));
  _m->DCU_IDS_DoorRR = ((_d[0] >> 3) & (0x01U));
  _m->DCU_IDS_DriverSeatBeltSt = (_d[1] & (0x03U));
  _m->DCU_IDS_KeySt = ((_d[1] >> 2) & (0x03U));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DCU_IDS_MSG207_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DCU_IDS_MSG207_testdb(&_m->mon1, DCU_IDS_MSG207_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return DCU_IDS_MSG207_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_DCU_IDS_MSG207_testdb(DCU_IDS_MSG207_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG207_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->DCU_IDS_DoorFL & (0x01U)) | ((_m->DCU_IDS_DoorFR & (0x01U)) << 1) | ((_m->DCU_IDS_DoorRL & (0x01U)) << 2) | ((_m->DCU_IDS_DoorRR & (0x01U)) << 3);
  cframe->Data[1] |= (_m->DCU_IDS_DriverSeatBeltSt & (0x03U)) | ((_m->DCU_IDS_KeySt & (0x03U)) << 2);

  cframe->MsgId = DCU_IDS_MSG207_CANID;
  cframe->DLC = DCU_IDS_MSG207_DLC;
  cframe->IDE = DCU_IDS_MSG207_IDE;
  return DCU_IDS_MSG207_CANID;
}

#else

uint32_t Pack_DCU_IDS_MSG207_testdb(DCU_IDS_MSG207_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG207_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->DCU_IDS_DoorFL & (0x01U)) | ((_m->DCU_IDS_DoorFR & (0x01U)) << 1) | ((_m->DCU_IDS_DoorRL & (0x01U)) << 2) | ((_m->DCU_IDS_DoorRR & (0x01U)) << 3);
  _d[1] |= (_m->DCU_IDS_DriverSeatBeltSt & (0x03U)) | ((_m->DCU_IDS_KeySt & (0x03U)) << 2);

  *_len = DCU_IDS_MSG207_DLC;
  *_ide = DCU_IDS_MSG207_IDE;
  return DCU_IDS_MSG207_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG208_testdb(DCU_IDS_MSG208_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DCU_IDS_RRMidDistance = (_d[0] & (0xFFU));
  _m->DCU_IDS_RLMidDistance = (_d[1] & (0xFFU));
  _m->DCU_IDS_RRDistance = (_d[2] & (0xFFU));
  _m->DCU_IDS_RLDistance = (_d[3] & (0xFFU));
  _m->DCU_IDS_RSLSideDistance = (_d[4] & (0xFFU));
  _m->DCU_IDS_RSRSideDistance = (_d[5] & (0xFFU));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DCU_IDS_MSG208_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DCU_IDS_MSG208_testdb(&_m->mon1, DCU_IDS_MSG208_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return DCU_IDS_MSG208_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_DCU_IDS_MSG208_testdb(DCU_IDS_MSG208_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG208_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->DCU_IDS_RRMidDistance & (0xFFU));
  cframe->Data[1] |= (_m->DCU_IDS_RLMidDistance & (0xFFU));
  cframe->Data[2] |= (_m->DCU_IDS_RRDistance & (0xFFU));
  cframe->Data[3] |= (_m->DCU_IDS_RLDistance & (0xFFU));
  cframe->Data[4] |= (_m->DCU_IDS_RSLSideDistance & (0xFFU));
  cframe->Data[5] |= (_m->DCU_IDS_RSRSideDistance & (0xFFU));

  cframe->MsgId = DCU_IDS_MSG208_CANID;
  cframe->DLC = DCU_IDS_MSG208_DLC;
  cframe->IDE = DCU_IDS_MSG208_IDE;
  return DCU_IDS_MSG208_CANID;
}

#else

uint32_t Pack_DCU_IDS_MSG208_testdb(DCU_IDS_MSG208_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG208_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->DCU_IDS_RRMidDistance & (0xFFU));
  _d[1] |= (_m->DCU_IDS_RLMidDistance & (0xFFU));
  _d[2] |= (_m->DCU_IDS_RRDistance & (0xFFU));
  _d[3] |= (_m->DCU_IDS_RLDistance & (0xFFU));
  _d[4] |= (_m->DCU_IDS_RSLSideDistance & (0xFFU));
  _d[5] |= (_m->DCU_IDS_RSRSideDistance & (0xFFU));

  *_len = DCU_IDS_MSG208_DLC;
  *_ide = DCU_IDS_MSG208_IDE;
  return DCU_IDS_MSG208_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG209_testdb(DCU_IDS_MSG209_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DCU_IDS_FRDistance = (_d[0] & (0xFFU));
  _m->DCU_IDS_FLDistance = (_d[1] & (0xFFU));
  _m->DCU_IDS_FRMidDistance = (_d[2] & (0xFFU));
  _m->DCU_IDS_FLMidDistance = (_d[3] & (0xFFU));
  _m->DCU_IDS_FSRSideDistance = (_d[4] & (0xFFU));
  _m->DCU_IDS_FSLSideDistance = (_d[5] & (0xFFU));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DCU_IDS_MSG209_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DCU_IDS_MSG209_testdb(&_m->mon1, DCU_IDS_MSG209_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return DCU_IDS_MSG209_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_DCU_IDS_MSG209_testdb(DCU_IDS_MSG209_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG209_DLC) && (i < 8); cframe->Data[i++] = 0);

  cframe->Data[0] |= (_m->DCU_IDS_FRDistance & (0xFFU));
  cframe->Data[1] |= (_m->DCU_IDS_FLDistance & (0xFFU));
  cframe->Data[2] |= (_m->DCU_IDS_FRMidDistance & (0xFFU));
  cframe->Data[3] |= (_m->DCU_IDS_FLMidDistance & (0xFFU));
  cframe->Data[4] |= (_m->DCU_IDS_FSRSideDistance & (0xFFU));
  cframe->Data[5] |= (_m->DCU_IDS_FSLSideDistance & (0xFFU));

  cframe->MsgId = DCU_IDS_MSG209_CANID;
  cframe->DLC = DCU_IDS_MSG209_DLC;
  cframe->IDE = DCU_IDS_MSG209_IDE;
  return DCU_IDS_MSG209_CANID;
}

#else

uint32_t Pack_DCU_IDS_MSG209_testdb(DCU_IDS_MSG209_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG209_DLC) && (i < 8); _d[i++] = 0);

  _d[0] |= (_m->DCU_IDS_FRDistance & (0xFFU));
  _d[1] |= (_m->DCU_IDS_FLDistance & (0xFFU));
  _d[2] |= (_m->DCU_IDS_FRMidDistance & (0xFFU));
  _d[3] |= (_m->DCU_IDS_FLMidDistance & (0xFFU));
  _d[4] |= (_m->DCU_IDS_FSRSideDistance & (0xFFU));
  _d[5] |= (_m->DCU_IDS_FSLSideDistance & (0xFFU));

  *_len = DCU_IDS_MSG209_DLC;
  *_ide = DCU_IDS_MSG209_IDE;
  return DCU_IDS_MSG209_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

uint32_t Unpack_DCU_IDS_MSG20C_testdb(DCU_IDS_MSG20C_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->DCU_IDS_CorrectedExterTemp_ro = (_d[0] & (0xFFU));
#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_CorrectedExterTemp_phys = (sigfloat_t)(TESTDB_DCU_IDS_CorrectedExterTemp_ro_fromS(_m->DCU_IDS_CorrectedExterTemp_ro));
#endif // TESTDB_USE_SIGFLOAT

  _m->DCU_IDS_CorrectedExterTempVD = (_d[1] & (0x01U));
  _m->DCU_IDS_TotalOdometer = ((_d[5] & (0x0FU)) << 16) | ((_d[6] & (0xFFU)) << 8) | (_d[7] & (0xFFU));

#ifdef TESTDB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DCU_IDS_MSG20C_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DCU_IDS_MSG20C_testdb(&_m->mon1, DCU_IDS_MSG20C_CANID);
#endif // TESTDB_USE_DIAG_MONITORS

  return DCU_IDS_MSG20C_CANID;
}

#ifdef TESTDB_USE_CANSTRUCT

uint32_t Pack_DCU_IDS_MSG20C_testdb(DCU_IDS_MSG20C_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG20C_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_CorrectedExterTemp_ro = TESTDB_DCU_IDS_CorrectedExterTemp_ro_toS(_m->DCU_IDS_CorrectedExterTemp_phys);
#endif // TESTDB_USE_SIGFLOAT

  cframe->Data[0] |= (_m->DCU_IDS_CorrectedExterTemp_ro & (0xFFU));
  cframe->Data[1] |= (_m->DCU_IDS_CorrectedExterTempVD & (0x01U));
  cframe->Data[5] |= ((_m->DCU_IDS_TotalOdometer >> 16) & (0x0FU));
  cframe->Data[6] |= ((_m->DCU_IDS_TotalOdometer >> 8) & (0xFFU));
  cframe->Data[7] |= (_m->DCU_IDS_TotalOdometer & (0xFFU));

  cframe->MsgId = DCU_IDS_MSG20C_CANID;
  cframe->DLC = DCU_IDS_MSG20C_DLC;
  cframe->IDE = DCU_IDS_MSG20C_IDE;
  return DCU_IDS_MSG20C_CANID;
}

#else

uint32_t Pack_DCU_IDS_MSG20C_testdb(DCU_IDS_MSG20C_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < DCU_IDS_MSG20C_DLC) && (i < 8); _d[i++] = 0);

#ifdef TESTDB_USE_SIGFLOAT
  _m->DCU_IDS_CorrectedExterTemp_ro = TESTDB_DCU_IDS_CorrectedExterTemp_ro_toS(_m->DCU_IDS_CorrectedExterTemp_phys);
#endif // TESTDB_USE_SIGFLOAT

  _d[0] |= (_m->DCU_IDS_CorrectedExterTemp_ro & (0xFFU));
  _d[1] |= (_m->DCU_IDS_CorrectedExterTempVD & (0x01U));
  _d[5] |= ((_m->DCU_IDS_TotalOdometer >> 16) & (0x0FU));
  _d[6] |= ((_m->DCU_IDS_TotalOdometer >> 8) & (0xFFU));
  _d[7] |= (_m->DCU_IDS_TotalOdometer & (0xFFU));

  *_len = DCU_IDS_MSG20C_DLC;
  *_ide = DCU_IDS_MSG20C_IDE;
  return DCU_IDS_MSG20C_CANID;
}

#endif // TESTDB_USE_CANSTRUCT

