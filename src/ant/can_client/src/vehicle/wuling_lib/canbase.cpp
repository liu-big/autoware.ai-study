
#include "vehicle/wuling_lib/canbase.hpp"


// --------------------------------------------------------------------------
uint32_t Unpack_MSG100_UserCAN2(MSG100_t *_m, const uint8_t *_d, uint8_t dlc_)
{
    _m->AutoCtrlEna = (_d[0] & (0x01U));
    _m->ModeCtrlCnt = (_d[6] & (0x0FU));
    _m->ModeCtrlCks = (_d[7] & (0xFFU));
    return MSG100_CANID;
}

uint32_t Pack_MSG100_UserCAN2(const MSG100_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide)
{
    uint8_t i;
    for (i = 0; i < MSG100_DLC; _d[i++] = 0)
        ;

    _d[0] |= (_m->AutoCtrlEna & (0x01U));
    _d[6] |= (_m->ModeCtrlCnt & (0x0FU));
    _d[7] |= (_m->ModeCtrlCks & (0xFFU));
    *_len = 8;
    *_ide = 0;
    return MSG100_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_MSG101_UserCAN2(MSG101_t *_m, const uint8_t *_d, uint8_t dlc_)
{
    _m->AccPedCmd = ((_d[0] & (0x03U)) << 8) | (_d[1] & (0xFFU));
    _m->AccTkoDis = ((_d[0] >> 6) & (0x01U));
    _m->AccCtrlEna = ((_d[0] >> 7) & (0x01U));
    _m->AccPedInv = ((_d[2] & (0x03U)) << 8) | (_d[3] & (0xFFU));
    _m->AccCtrlCnt = (_d[6] & (0x0FU));
    _m->AccCtrlCks = (_d[7] & (0xFFU));
    return MSG101_CANID;
}

uint32_t Pack_MSG101_UserCAN2(const MSG101_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide)
{
    uint8_t i;
    for (i = 0; i < MSG101_DLC; _d[i++] = 0)
        ;

    _d[0] |= ((_m->AccPedCmd >> 8) & (0x03U)) | ((_m->AccTkoDis & (0x01U)) << 6) | ((_m->AccCtrlEna & (0x01U)) << 7);
    _d[1] |= (_m->AccPedCmd & (0xFFU));
    _d[2] |= ((_m->AccPedInv >> 8) & (0x03U));
    _d[3] |= (_m->AccPedInv & (0xFFU));
    _d[6] |= (_m->AccCtrlCnt & (0x0FU));
    _d[7] |= (_m->AccCtrlCks & (0xFFU));
    *_len = 8;
    *_ide = 0;
    return MSG101_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_MSG102_UserCAN2(MSG102_t *_m, const uint8_t *_d, uint8_t dlc_)
{
    _m->BrkPedCmd = ((_d[0] & (0x03U)) << 8) | (_d[1] & (0xFFU));
    _m->BrkTkoDis = ((_d[0] >> 6) & (0x01U));
    _m->BrkCtrlEna = ((_d[0] >> 7) & (0x01U));
    _m->BrkPedInv = ((_d[2] & (0x03U)) << 8) | (_d[3] & (0xFFU));
    _m->BrkCtrlCnt = (_d[6] & (0x0FU));
    _m->BrkCtrlCks = (_d[7] & (0xFFU));
    return MSG102_CANID;
}

uint32_t Pack_MSG102_UserCAN2(const MSG102_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide)
{
    uint8_t i;
    for (i = 0; i < MSG102_DLC; _d[i++] = 0)
        ;

    _d[0] |= ((_m->BrkPedCmd >> 8) & (0x03U)) | ((_m->BrkTkoDis & (0x01U)) << 6) | ((_m->BrkCtrlEna & (0x01U)) << 7);
    _d[1] |= (_m->BrkPedCmd & (0xFFU));
    _d[2] |= ((_m->BrkPedInv >> 8) & (0x03U));
    _d[3] |= (_m->BrkPedInv & (0xFFU));
    _d[6] |= (_m->BrkCtrlCnt & (0x0FU));
    _d[7] |= (_m->BrkCtrlCks & (0xFFU));
    *_len = 8;
    *_ide = 0;
    return MSG102_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_MSG103_UserCAN2(MSG103_t *_m, const uint8_t *_d, uint8_t dlc_)
{
    _m->StrAngCmd = (((_d[0] & (0x3FU)) << 8) | (_d[1] & (0xFFU))) - 8000;
    _m->StrCtrlEna = ((_d[0] >> 7) & (0x01U));
    _m->StrAngLimit = (((_d[2] & (0x3FU)) << 8) | (_d[3] & (0xFFU))) - 8000;
    _m->StrAngRateLimit = ((_d[4] & (0x0FU)) << 8) | (_d[5] & (0xFFU));
    _m->StrCtrlCnt = (_d[6] & (0x0FU));
    _m->StrCtrlCks = (_d[7] & (0xFFU));
    return MSG103_CANID;
}

uint32_t Pack_MSG103_UserCAN2(const MSG103_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide)
{
    uint8_t i;
    for (i = 0; i < MSG103_DLC; _d[i++] = 0)
        ;

    _d[0] |= (((_m->StrAngCmd+8000) >> 8) & (0x3FU)) | ((_m->StrCtrlEna & (0x01U)) << 7);
    _d[1] |= ((_m->StrAngCmd+8000) & (0xFFU));
    _d[2] |= (((_m->StrAngLimit+8000) >> 8) & (0x3FU));
    _d[3] |= ((_m->StrAngLimit+8000) & (0xFFU));
    _d[4] |= ((_m->StrAngRateLimit >> 8) & (0x0FU));
    _d[5] |= (_m->StrAngRateLimit & (0xFFU));
    _d[6] |= (_m->StrCtrlCnt & (0x0FU));
    _d[7] |= (_m->StrCtrlCks & (0xFFU));
    *_len = 8;
    *_ide = 0;
    return MSG103_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_MSG104_UserCAN2(MSG104_t *_m, const uint8_t *_d, uint8_t dlc_)
{
    _m->GearCmd = (_d[0] & (0x0FU));
    _m->GearCtrlEna = ((_d[0] >> 7) & (0x01U));
    _m->TurnLightCmd = (_d[1] & (0x03U));
    _m->BeamLightCmd = ((_d[1] >> 2) & (0x03U));
    _m->HornCmd = ((_d[1] >> 4) & (0x01U));
    _m->GearCtrlCnt = (_d[6] & (0x0FU));
    _m->GearCtrlCks = (_d[7] & (0xFFU));
    return MSG104_CANID;
}

uint32_t Pack_MSG104_UserCAN2(const MSG104_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide)
{
    uint8_t i;
    for (i = 0; i < MSG104_DLC; _d[i++] = 0)
        ;

    _d[0] |= (_m->GearCmd & (0x0FU)) | ((_m->GearCtrlEna & (0x01U)) << 7);
    _d[1] |= (_m->TurnLightCmd & (0x03U)) | ((_m->BeamLightCmd & (0x03U)) << 2) | ((_m->HornCmd & (0x01U)) << 4);
    _d[6] |= (_m->GearCtrlCnt & (0x0FU));
    _d[7] |= (_m->GearCtrlCks & (0xFFU));
    *_len = 8;
    *_ide = 0;
    return MSG104_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_MSG200_UserCAN2(MSG200_t *_m, const uint8_t *_d, uint8_t dlc_)
{
    _m->AutoCtrlStat = (_d[0] & (0x03U));
    _m->ModeStatCnt = (_d[6] & (0x0FU));
    _m->ModeStatCks = (_d[7] & (0xFFU));
    return MSG200_CANID;
}

uint32_t Pack_MSG200_UserCAN2(const MSG200_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide)
{
    uint8_t i;
    for (i = 0; i < MSG200_DLC; _d[i++] = 0)
        ;

    _d[0] |= (_m->AutoCtrlStat & (0x03U));
    _d[6] |= (_m->ModeStatCnt & (0x0FU));
    _d[7] |= (_m->ModeStatCks & (0xFFU));
    *_len = 8;
    *_ide = 0;
    return MSG200_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_MSG201_UserCAN2(MSG201_t *_m, const uint8_t *_d, uint8_t dlc_)
{
    _m->AccPedAct = ((_d[0] & (0x03U)) << 8) | (_d[1] & (0xFFU));
    _m->AccCtrlStat = ((_d[0] >> 6) & (0x03U));
    _m->AccPedCmd = ((_d[2] & (0x03U)) << 8) | (_d[3] & (0xFFU));
    _m->AccPedExe = ((_d[4] & (0x03U)) << 8) | (_d[5] & (0xFFU));
    _m->AccStatCnt = (_d[6] & (0x0FU));
    _m->AccStatCks = (_d[7] & (0xFFU));
    return MSG201_CANID;
}

uint32_t Pack_MSG201_UserCAN2(const MSG201_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide)
{
    uint8_t i;
    for (i = 0; i < MSG201_DLC; _d[i++] = 0)
        ;

    _d[0] |= ((_m->AccPedAct >> 8) & (0x03U)) | ((_m->AccCtrlStat & (0x03U)) << 6);
    _d[1] |= (_m->AccPedAct & (0xFFU));
    _d[2] |= ((_m->AccPedCmd >> 8) & (0x03U));
    _d[3] |= (_m->AccPedCmd & (0xFFU));
    _d[4] |= ((_m->AccPedExe >> 8) & (0x03U));
    _d[5] |= (_m->AccPedExe & (0xFFU));
    _d[6] |= (_m->AccStatCnt & (0x0FU));
    _d[7] |= (_m->AccStatCks & (0xFFU));
    *_len = 8;
    *_ide = 0;
    return MSG201_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_MSG202_UserCAN2(MSG202_t *_m, const uint8_t *_d, uint8_t dlc_)
{
    _m->BrkPedAct = ((_d[0] & (0x03U)) << 8) | (_d[1] & (0xFFU));
    _m->BrkCtrlStat = ((_d[0] >> 6) & (0x03U));
    _m->BrkPedCmd = ((_d[2] & (0x03U)) << 8) | (_d[3] & (0xFFU));
    _m->BrkPedExe = ((_d[4] & (0x03U)) << 8) | (_d[5] & (0xFFU));
    _m->BrkStatCnt = (_d[6] & (0x0FU));
    _m->BrkStatCks = (_d[7] & (0xFFU));
    return MSG202_CANID;
}

uint32_t Pack_MSG202_UserCAN2(const MSG202_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide)
{
    uint8_t i;
    for (i = 0; i < MSG202_DLC; _d[i++] = 0)
        ;

    _d[0] |= ((_m->BrkPedAct >> 8) & (0x03U)) | ((_m->BrkCtrlStat & (0x03U)) << 6);
    _d[1] |= (_m->BrkPedAct & (0xFFU));
    _d[2] |= ((_m->BrkPedCmd >> 8) & (0x03U));
    _d[3] |= (_m->BrkPedCmd & (0xFFU));
    _d[4] |= ((_m->BrkPedExe >> 8) & (0x03U));
    _d[5] |= (_m->BrkPedExe & (0xFFU));
    _d[6] |= (_m->BrkStatCnt & (0x0FU));
    _d[7] |= (_m->BrkStatCks & (0xFFU));
    *_len = 8;
    *_ide = 0;
    return MSG202_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_MSG203_UserCAN2(MSG203_t *_m, const uint8_t *_d, uint8_t dlc_)
{
    _m->StrAngAct = (((_d[0] & (0x3FU)) << 8) | (_d[1] & (0xFFU))) - 8000;
    _m->StrCtrlStat = ((_d[0] >> 6) & (0x03U));
    _m->StrAngCmd = (((_d[2] & (0x3FU)) << 8) | (_d[3] & (0xFFU))) - 8000;
    _m->StrTrqAct = ((_d[4] & (0x7FU))) - 64;
    _m->StrStatCnt = (_d[6] & (0x0FU));
    _m->StrStatCks = (_d[7] & (0xFFU));
    return MSG203_CANID;
}

uint32_t Pack_MSG203_UserCAN2(const MSG203_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide)
{
    uint8_t i;
    for (i = 0; i < MSG203_DLC; _d[i++] = 0)
        ;

    _d[0] |= ((_m->StrAngAct >> 8) & (0x3FU)) | ((_m->StrCtrlStat & (0x03U)) << 6);
    _d[1] |= (_m->StrAngAct & (0xFFU));
    _d[2] |= ((_m->StrAngCmd >> 8) & (0x3FU));
    _d[3] |= (_m->StrAngCmd & (0xFFU));
    _d[4] |= (_m->StrTrqAct & (0x7FU));
    _d[6] |= (_m->StrStatCnt & (0x0FU));
    _d[7] |= (_m->StrStatCks & (0xFFU));
    *_len = 8;
    *_ide = 0;
    return MSG203_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_MSG204_UserCAN2(MSG204_t *_m, const uint8_t *_d, uint8_t dlc_)
{
    _m->GearAct = (_d[0] & (0x0FU));
    _m->GearCmd = ((_d[0] >> 4) & (0x0FU));
    _m->TurnLightAct = (_d[1] & (0x03U));
    _m->TurnLightCmd = ((_d[1] >> 2) & (0x03U));
    _m->BeamLightAct = ((_d[1] >> 4) & (0x03U));
    _m->BeamLightCmd = ((_d[1] >> 6) & (0x03U));
    _m->HornStatus = ((_d[2] >> 6) & (0x01U));
    _m->HornCmd = ((_d[2] >> 7) & (0x01U));
    _m->VehSpd = ((_d[3] & (0x1FU)) << 8) | (_d[4] & (0xFFU));
    _m->GearStatCnt = (_d[6] & (0x0FU));
    _m->GearStatCks = (_d[7] & (0xFFU));
    return MSG204_CANID;
}

uint32_t Pack_MSG204_UserCAN2(const MSG204_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide)
{
    uint8_t i;
    for (i = 0; i < MSG204_DLC; _d[i++] = 0)
        ;

    _d[0] |= (_m->GearAct & (0x0FU)) | ((_m->GearCmd & (0x0FU)) << 4);
    _d[1] |= (_m->TurnLightAct & (0x03U)) | ((_m->TurnLightCmd & (0x03U)) << 2) | ((_m->BeamLightAct & (0x03U)) << 4) | ((_m->BeamLightCmd & (0x03U)) << 6);
    _d[2] |= ((_m->HornStatus & (0x01U)) << 6) | ((_m->HornCmd & (0x01U)) << 7);
    _d[6] |= (_m->GearStatCnt & (0x0FU));
    _d[7] |= (_m->GearStatCks & (0xFFU));
    *_len = 8;
    *_ide = 0;
    return MSG204_CANID;
}
