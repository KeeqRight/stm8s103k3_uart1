#ifndef __RN8209D_H__
#define __RN8209D_H__


#include "stm32f0xx.h"


#define WREN 0xEA

#define Reg_SYSCON    0x00
#define Reg_EMUCON    0x01
#define Reg_HFCONST   0x02
#define Reg_PStart    0x03
#define Reg_DStart    0x04
#define Reg_GPQA      0x05
#define Reg_GPQB      0x06
#define Reg_PhsA      0x07
#define Reg_PhsB      0x08
#define Reg_QPhsCal   0x09
#define Reg_APOSA     0x0A
#define Reg_APOSB     0x0B
#define Reg_RPOSA     0x0C
#define Reg_RPOSB     0x0D
#define Reg_IARMSOS   0x0E
#define Reg_IBRMSOS   0x0F
#define Reg_IBGain    0x10
#define Reg_D2FPL     0x11
#define Reg_D2FPH     0x12
#define Reg_DCIAH     0x13
#define Reg_DCIBH     0x14
#define Reg_DCUH      0x15
#define Reg_DCL       0x16
#define Reg_EMUCON2   0x17

#define Reg_PFCnt     0x20
#define Reg_DFCnt     0x21
#define Reg_IARMS     0x22
#define Reg_IBRMS     0x23
#define Reg_URMS      0x24
#define Reg_UFreq     0x25
#define Reg_PowerPA   0x26
#define Reg_PowerPB   0x27
#define Reg_PowerQ    0x28
#define Reg_EnergyP   0x29
#define Reg_EnergyP2  0x2A
#define Reg_EnergyD   0x2B
#define Reg_EnergyD2  0x2C
#define Reg_EMUStatus 0x2D

#define Reg_SPL_IA    0x30
#define Reg_SPL_IB    0x31
#define Reg_SPL_U     0x32

#define Reg_UFreq2    0x35

#define Reg_IE        0x40
#define Reg_IF        0x41
#define Reg_RIF       0x42

#define Reg_SysStatus 0x43
#define Reg_RData     0x44
#define Reg_WData     0x45

#define Reg_DeviceID  0x7F


#define phase_A      0x00
#define phase_B      0x01

typedef struct{
    uint16_t Cst_HFConst;
    uint16_t Cst_PStart;  //有功启动功率
    uint16_t Cst_DStart;
    uint16_t Cst_GPQA;    //A通道功率增益校正
    uint16_t Cst_GPQB;    //B通道功率增益校正
    uint8_t  Cst_PhsA;    //A通道相位校正
    uint8_t  Cst_PhsB;    //B通道相位校正
    uint16_t Cst_QPhsCal; //无功相位校正
    uint16_t Cst_APOSA;   //A通道有功功率offset校正
    uint16_t Cst_APOSB;   //B通道有功功率offset校正
    uint16_t Cst_RPOSA;   //A通道无功功率offset校正
    uint16_t Cst_RPOSB;   //B通道无功功率offset校正
    uint16_t Cst_IARMSOS; //A通道电流有效值offset校正
    uint16_t Cst_IBRMSOS; //B通道电流有效值offset校正
    uint16_t Cst_IBGain;  //B通道电流增益

    uint16_t Cst_Ku;
    uint32_t Cst_Kia;
    uint32_t Cst_Kib;
    uint16_t Cst_Kpa;
    uint16_t Cst_Kpb;

    uint16_t cheskSum;

}StDef_RN8209DPara;

extern StDef_RN8209DPara StDef_RN8209DPara_Reg;

uint8_t RN8209D_Init(void);
void    RN8209D_CalibrateInit(void);
void    RN8209D_CalibrateGPQxErr(uint8_t phase,int16_t err);
void    RN8209D_CalibratePhsXErr(uint8_t phase,int16_t err);
void    RN8209D_CalibrateAPOSx(uint8_t phase);
void    RN8209D_CalibrateCurrentOffset(uint8_t phase);
void    RN8209D_CalibrateKx(uint8_t phase);
uint8_t RN8209D_ReadVoltage(uint16_t *vol);
uint8_t RN8209D_ReadCurrent(uint8_t phase,uint16_t *current);
uint8_t RN8209D_ReadPower(uint8_t phase, uint32_t *p);
uint8_t RN8209D_ReadTotalE(uint8_t phase,uint32_t *p);
uint8_t RN8209D_ReadCheckSum(uint32_t *p);
#endif

