#include "RN8209D.h"
#include "platform_config.h"
#include "delay.h"
#include "stm32f0xx_gpio.h"
#include "global.h"
#include <math.h>  //求根号

#define USING_SOFT_SPI


#define RN8209D_CS_SET()    GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define RN8209D_CS_CLR()    GPIO_ResetBits(GPIOB,GPIO_Pin_5)

#define RN8209D_SCK_SET()   GPIO_SetBits(GPIOD,GPIO_Pin_2)
#define RN8209D_SCK_CLR()   GPIO_ResetBits(GPIOD,GPIO_Pin_2)

#define RN8209D_Read_MISO() GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)

#define RN8209D_MOSI_SET()  GPIO_SetBits(GPIOB,GPIO_Pin_4)
#define RN8209D_MOSI_CLR()  GPIO_ResetBits(GPIOB,GPIO_Pin_4)

#ifdef USE_SMARTACU110
    #define RN8209D_RST_SET()  GPIO_SetBits(GPIOC,GPIO_Pin_10)
    #define RN8209D_RST_CLR()  GPIO_ResetBits(GPIOC,GPIO_Pin_10)
#endif

#ifdef USE_SMARTACU120
    #define RN8209D_RST_SET()  GPIO_SetBits(GPIOC,GPIO_Pin_7)
    #define RN8209D_RST_CLR()  GPIO_ResetBits(GPIOC,GPIO_Pin_7)
#endif

#define RNDelay         delay_us
#define RN_CLOCKWIDTH   30


StDef_RN8209DPara StDef_RN8209DPara_Reg;

void RN8209D_GetCheckSum(void);

void RN8209D_ReadRegNoCheck(uint8_t addr,uint8_t *regbuf,uint8_t regbuflen)
{
    uint8_t ucI,ucK;

    RN8209D_CS_CLR();
    delay_us(10);
    addr |= 0x00;            //发送读数据命令(bit.7=0)
    for(ucI=0;ucI<8;ucI++)	//发送读数据命令,下降沿接收数据,高位在前，低位在后
    {
        if( addr & 0x80 )
            RN8209D_MOSI_SET();
        else
            RN8209D_MOSI_CLR();
        RN8209D_SCK_SET();
        delay_us(RN_CLOCKWIDTH);
        RN8209D_SCK_CLR();      //在时钟的下降沿写数据
        addr<<=1;
        delay_us(RN_CLOCKWIDTH);
    }
    RN8209D_MOSI_CLR();
    RN8209D_SCK_CLR();
    delay_us(50);
    for(ucK=0;ucK<regbuflen;ucK++)
    {
        regbuf[ucK]=0x00;
        for(ucI=0;ucI<8;ucI++)
        {
            RN8209D_SCK_SET();
            delay_us(RN_CLOCKWIDTH);
            regbuf[ucK]<<=1;
            RN8209D_SCK_CLR();      //在时钟的下降沿读数据
            delay_us(RN_CLOCKWIDTH/2);
            if(RN8209D_Read_MISO())
                regbuf[ucK]=regbuf[ucK]|0x01;
            delay_us(RN_CLOCKWIDTH/2);
        }
    }
    RN8209D_CS_SET();
    delay_us(10);
    RN8209D_SCK_CLR();
    delay_us(10);
}

uint8_t RN8209D_ReadReg(uint8_t addr,uint8_t *regbuf,uint8_t regbuflen)
{
    uint8_t buf[4];

    RN8209D_ReadRegNoCheck(addr,regbuf,regbuflen);
    m_memset(buf,0x01,4);
    RN8209D_ReadRegNoCheck(Reg_RData,buf,4);

    if(regbuflen == 3){
        if(m_memcmp(regbuf,buf+1,regbuflen) == 0)
            return 0;
        else
            return 1;
    }
    else if(regbuflen == 4){
        if(m_memcmp(regbuf,buf,regbuflen) == 0)
            return 0;
        else
            return 1;
    }
    else if(regbuflen == 2){
        if(m_memcmp(regbuf,buf+2,regbuflen) == 0)
            return 0;
        else
            return 1;
    }
    else{
        return 1;
    }

}

void RN8209D_WriteRegNoCheck(uint8_t addr,uint8_t *regbuf,uint8_t regbuflen)
{
    uint8_t ucI,ucK;

    RN8209D_CS_CLR();
    delay_us(100);
    addr |= 0x80;   //发送写数据命令(bit.7=1)
    for(ucI=0;ucI<8;ucI++)
    {
        if( addr & 0x80 )
            RN8209D_MOSI_SET();
        else
            RN8209D_MOSI_CLR(); ;
        RN8209D_SCK_SET();
        delay_us(RN_CLOCKWIDTH);
        RN8209D_SCK_CLR();          //在时钟的下降沿写数据
        addr<<=1;
        delay_us(RN_CLOCKWIDTH);
    }
    RN8209D_MOSI_CLR();
    RN8209D_SCK_CLR();
    delay_us(500);
    for(ucK=0;ucK<regbuflen;ucK++)
    {
        for(ucI=0;ucI<8;ucI++)
        {
            if(regbuf[ucK] & 0x80 )
                RN8209D_MOSI_SET();
            else
                RN8209D_MOSI_CLR();
            RN8209D_SCK_SET();
            delay_us(RN_CLOCKWIDTH);
            RN8209D_SCK_CLR();      //在时钟的下降沿写数据
            regbuf[ucK]<<=1;
            delay_us(RN_CLOCKWIDTH);
        }
    }
    RN8209D_MOSI_CLR();
    delay_us(100);
    RN8209D_CS_SET();
    delay_us(100);
    RN8209D_SCK_CLR();
    delay_us(100);
}

uint8_t RN8209D_WriteReg(uint8_t addr,uint8_t *regbuf,uint8_t regbuflen)
{
    uint8_t buf[4];

    RN8209D_WriteRegNoCheck(addr,regbuf,regbuflen);
    m_memset(buf,0,4);
    RN8209D_WriteRegNoCheck(Reg_WData,buf,regbuflen);

    if(regbuflen == 3){
        if(m_memcmp(regbuf,buf+1,regbuflen) == 0)
            return 0;
        else
            return 1;
    }
    else if(regbuflen == 4){
        if(m_memcmp(regbuf,buf,regbuflen) == 0)
            return 0;
        else
            return 1;
    }
    else if(regbuflen == 2){
        if(m_memcmp(regbuf,buf+2,regbuflen) == 0)
            return 0;
        else
            return 1;
    }
    else{
        return 1;
    }
}

static void RN8209D_GPIOConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOC|RCC_AHBPeriph_GPIOD,ENABLE);

    /* SCLK */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_NOPULL;//;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* CS MOSI */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

#ifdef USE_SMARTACU110
    /* RST */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_SetBits(GPIOC,GPIO_Pin_10);            //没有这条语句，会导致初始化后端口输出电平为0
    GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#ifdef USE_SMARTACU120
    /* RST */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

    /* MISO */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/********************************************************
功能描述：   读取00H~17H寄存器校验和
参数说明：
返回说明：
调用方式：
全局变量：
读写时间：
注意事项：
日期    ：
********************************************************/
void RN8209D_GetCheckSum(void)
{

    uint16_t regbuf[24],i;

    //00H 系统控制寄存器
    regbuf[0] = 0x0051;
    //01H 计量控制
    regbuf[1] = 0x8003;  //电能读后清零
    //02H 写HFCONST
    regbuf[2] = 0x357B;
    //03H,04H启动功率
    regbuf[3] = 0x012D;
    regbuf[4] = 0x012D;
    //05H A通道有功功率增益;
    regbuf[5] = StDef_RN8209DPara_Reg.Cst_GPQA;
    //06HB通道有功功率增益
    regbuf[6] = StDef_RN8209DPara_Reg.Cst_GPQB;
    //07H A通道相位校正
    regbuf[7] = StDef_RN8209DPara_Reg.Cst_PhsA;
    //08 B通道相位校正
    regbuf[8] = StDef_RN8209DPara_Reg.Cst_PhsB;
    //09 无功相位补偿
    regbuf[9] = 0;
    //0AH A通道有功功率offset
    regbuf[10] = StDef_RN8209DPara_Reg.Cst_APOSA;
    //0BH B通道有功功率offset
    regbuf[11] = StDef_RN8209DPara_Reg.Cst_APOSB;
    //0CH A通道无功功率offset
    regbuf[12] = StDef_RN8209DPara_Reg.Cst_RPOSA;
    //0DH B通道无功功率offset
    regbuf[13] = StDef_RN8209DPara_Reg.Cst_RPOSB;
    //0EH A通道电流offset
    regbuf[14] = StDef_RN8209DPara_Reg.Cst_IARMSOS;
    //0FH B通道电流offset
    regbuf[15] = StDef_RN8209DPara_Reg.Cst_IBRMSOS;
    //10H B通道电流增益
    regbuf[16] = StDef_RN8209DPara_Reg.Cst_IBGain;
    //11H 无功相位补偿
    regbuf[17] = 0;
    //12H 无功相位补偿
    regbuf[18] = 0;
    //13H 无功相位补偿
    regbuf[19] = 0;
    //14H 无功相位补偿
    regbuf[20] = 0;
    //15H 无功相位补偿
    regbuf[21] = 0;
    //16H 无功相位补偿
    regbuf[22] = 0;
    //17H 计量控制2
    regbuf[23] = 0x00B0;

    StDef_RN8209DPara_Reg.cheskSum = 0;
    for(i=0;i<24;i++){
        StDef_RN8209DPara_Reg.cheskSum += regbuf[i];
    }
    StDef_RN8209DPara_Reg.cheskSum = ~StDef_RN8209DPara_Reg.cheskSum;
}


uint8_t RN8209D_Init(void)
{
    uint8_t regbuf[4];
    uint8_t status = 1;

    RN8209D_GPIOConfig();

    RN8209D_RST_CLR();
    delay_ms(100);
    RN8209D_RST_SET();
    delay_ms(100);

    //读取RN8209 DeviceID
    if(RN8209D_ReadReg(Reg_DeviceID,regbuf,3) == 0)
    {
        if((regbuf[0]==0x82)&&(regbuf[1]==0x09)&&(regbuf[2]==0x00)){
            status = 0;
        }
        else{
            status = 1;
        }
    }

    //默认换算系数
    //StDef_RN8209DPara_Reg.Cst_Kia = 41938;
    //StDef_RN8209DPara_Reg.Cst_Kib = 44529;
    //StDef_RN8209DPara_Reg.Cst_Ku  = 8711;

    if(status == 0)
    {
        //写使能
        regbuf[0] = 0xE5;
        RN8209D_WriteReg(WREN,regbuf,1);
        //复位
        regbuf[0] = 0xFA;
        RN8209D_WriteReg(WREN,regbuf,1);
        delay_ms(20);
        //写使能
        regbuf[0] = 0xE5;
        RN8209D_WriteReg(WREN,regbuf,1);
        //系统控制寄存器
        regbuf[0] = 0x00;
        regbuf[1] = 0x51;  //开启通道B,A、B通道增益2
        RN8209D_WriteReg(Reg_SYSCON,regbuf,2);
        m_memset(regbuf,0,2);
        RN8209D_ReadReg(Reg_SYSCON,regbuf,2);
        //写HFCONST
        regbuf[0] = 0x35;
        regbuf[1] = 0x7B;
        RN8209D_WriteReg(Reg_HFCONST,regbuf,2);
        m_memset(regbuf,0,2);
        RN8209D_ReadReg(Reg_HFCONST,regbuf,2);
        //启动功率
        regbuf[0] = 0x01;
        regbuf[1] = 0x2D;
        RN8209D_WriteReg(Reg_PStart,regbuf,2);
        regbuf[0] = 0x01;
        regbuf[1] = 0x2D;
        RN8209D_WriteReg(Reg_DStart,regbuf,2);
        //计量控制
        regbuf[0] = 0x80;  //电能读后清零
        regbuf[1] = 0x03;
        RN8209D_WriteReg(Reg_EMUCON,regbuf,2);
        //计量控制2
        regbuf[0] = 0x00;
        regbuf[1] = 0xB0;
        RN8209D_WriteReg(Reg_EMUCON2,regbuf,2);
        //A通道有功功率增益
        //StDef_RN8209DPara_Reg.Cst_GPQA = 0;
        regbuf[0] = StDef_RN8209DPara_Reg.Cst_GPQA/256;
        regbuf[1] = StDef_RN8209DPara_Reg.Cst_GPQA%256;
        RN8209D_WriteReg(Reg_GPQA,regbuf,2);
        //B通道有功功率增益
        //StDef_RN8209DPara_Reg.Cst_GPQB = 0;
        regbuf[0] = StDef_RN8209DPara_Reg.Cst_GPQB/256;
        regbuf[1] = StDef_RN8209DPara_Reg.Cst_GPQB%256;
        RN8209D_WriteReg(Reg_GPQB,regbuf,2);
        //A通道相位校正
        //StDef_RN8209DPara_Reg.Cst_PhsA = 0;
        regbuf[0] = StDef_RN8209DPara_Reg.Cst_PhsA;
        RN8209D_WriteReg(Reg_PhsA,regbuf,1);
        //B通道相位校正
        //StDef_RN8209DPara_Reg.Cst_PhsB = 0;
        regbuf[0] = StDef_RN8209DPara_Reg.Cst_PhsB;
        RN8209D_WriteReg(Reg_PhsB,regbuf,1);
        //A通道有功功率offset
        //StDef_RN8209DPara_Reg.Cst_APOSA = 0;
        regbuf[0] = StDef_RN8209DPara_Reg.Cst_APOSA/256;
        regbuf[1] = StDef_RN8209DPara_Reg.Cst_APOSA%256;
        RN8209D_WriteReg(Reg_APOSA,regbuf,2);
        //B通道有功功率offset
        //StDef_RN8209DPara_Reg.Cst_APOSB = 0;
        regbuf[0] = StDef_RN8209DPara_Reg.Cst_APOSB/256;
        regbuf[1] = StDef_RN8209DPara_Reg.Cst_APOSB%256;
        RN8209D_WriteReg(Reg_APOSB,regbuf,2);
        //A通道无功功率offset
        StDef_RN8209DPara_Reg.Cst_RPOSA = 0;
        regbuf[0] = StDef_RN8209DPara_Reg.Cst_RPOSA/256;
        regbuf[1] = StDef_RN8209DPara_Reg.Cst_RPOSA%256;
        RN8209D_WriteReg(Reg_RPOSA,regbuf,2);
        //B通道无功功率offset
        StDef_RN8209DPara_Reg.Cst_RPOSB = 0;
        regbuf[0] = StDef_RN8209DPara_Reg.Cst_RPOSB/256;
        regbuf[1] = StDef_RN8209DPara_Reg.Cst_RPOSB%256;
        RN8209D_WriteReg(Reg_RPOSB,regbuf,2);
        //A通道电流offset
        //StDef_RN8209DPara_Reg.Cst_IARMSOS = 0;
        regbuf[0] = StDef_RN8209DPara_Reg.Cst_IARMSOS/256;
        regbuf[1] = StDef_RN8209DPara_Reg.Cst_IARMSOS%256;
        RN8209D_WriteReg(Reg_IARMSOS,regbuf,2);
        //B通道电流offset
        //StDef_RN8209DPara_Reg.Cst_IBRMSOS = 0;
        regbuf[0] = StDef_RN8209DPara_Reg.Cst_IBRMSOS/256;
        regbuf[1] = StDef_RN8209DPara_Reg.Cst_IBRMSOS%256;
        RN8209D_WriteReg(Reg_IBRMSOS,regbuf,2);
        //B通道电流增益
        StDef_RN8209DPara_Reg.Cst_IBGain = 0;
        regbuf[0] = StDef_RN8209DPara_Reg.Cst_IBGain/256;
        regbuf[1] = StDef_RN8209DPara_Reg.Cst_IBGain%256;
        RN8209D_WriteReg(Reg_IBGain,regbuf,2);
        //直流通道
        regbuf[0]=0;regbuf[1]=0;
        RN8209D_WriteReg(Reg_D2FPL,regbuf,2);
        regbuf[0]=0;regbuf[1]=0;
        RN8209D_WriteReg(Reg_D2FPH,regbuf,2);
        regbuf[0]=0;regbuf[1]=0;
        RN8209D_WriteReg(Reg_DCIAH,regbuf,2);
        regbuf[0]=0;regbuf[1]=0;
        RN8209D_WriteReg(Reg_DCIBH,regbuf,2);
        regbuf[0]=0;regbuf[1]=0;
        RN8209D_WriteReg(Reg_DCUH,regbuf,2);
        regbuf[0]=0;regbuf[1]=0;
        RN8209D_WriteReg(Reg_DCL,regbuf,2);
        //写保护
        regbuf[0] = 0xDC;
        RN8209D_WriteReg(WREN,regbuf,1);
        //获取校表寄存器校验值
        RN8209D_GetCheckSum();
    }

    return status;
}

void RN8209D_CalibrateInit(void)
{
    uint8_t regbuf[4];
    uint8_t status = 1;

    //读取RN8209 DeviceID
    if(RN8209D_ReadReg(Reg_DeviceID,regbuf,3) == 0){
        if((regbuf[0]==0x82)&&(regbuf[1]==0x09)&&(regbuf[2]==0x00)){
            status = 0;
        }
        else{
            status = 1;
        }
    }

    //默认换算系数
    //StDef_RN8209DPara_Reg.Cst_Kia = 41938;
    //StDef_RN8209DPara_Reg.Cst_Kib = 44529;
    //StDef_RN8209DPara_Reg.Cst_Ku = 8711;
    if(status == 0){
        //写使能
        regbuf[0] = 0xE5;
        RN8209D_WriteReg(WREN,regbuf,1);
        //复位
        regbuf[0] = 0xFA;
        RN8209D_WriteReg(WREN,regbuf,1);
        delay_ms(20);
        //写使能
        regbuf[0] = 0xE5;
        RN8209D_WriteReg(WREN,regbuf,1);
        //系统控制寄存器
        regbuf[0] = 0x00;
        regbuf[1] = 0x51;  //开启通道B
        RN8209D_WriteReg(Reg_SYSCON,regbuf,2);
        m_memset(regbuf,0,2);
        RN8209D_ReadReg(Reg_SYSCON,regbuf,2);
        //写HFCONST
        regbuf[0] = 0x35;
        regbuf[1] = 0x7B;
        RN8209D_WriteReg(Reg_HFCONST,regbuf,2);
        m_memset(regbuf,0,2);
        RN8209D_ReadReg(Reg_HFCONST,regbuf,2);
        //启动功率
        regbuf[0] = 0x01;
        regbuf[1] = 0x2D;
        RN8209D_WriteReg(Reg_PStart,regbuf,2);
        regbuf[0] = 0x01;
        regbuf[1] = 0x2D;
        RN8209D_WriteReg(Reg_DStart,regbuf,2);
        //计量控制
        regbuf[0] = 0x80;  //电能读后清零
        regbuf[1] = 0x03;
        RN8209D_WriteReg(Reg_EMUCON,regbuf,2);
        //计量控制2
        regbuf[0] = 0x00;
        regbuf[1] = 0xB0;  //自定义电能寄存器为B通道
        RN8209D_WriteReg(Reg_EMUCON2,regbuf,2);
        //B通道电流增益
        regbuf[0] = 0;
        regbuf[1] = 0;
        RN8209D_WriteReg(Reg_IBGain,regbuf,2);
        //A通道有功功率增益
        regbuf[0] = 0;
        regbuf[1] = 0;
        RN8209D_WriteReg(Reg_GPQA,regbuf,2);
        //B通道有功功率增益
        regbuf[0] = 0;
        regbuf[1] = 0;
        RN8209D_WriteReg(Reg_GPQB,regbuf,2);
        //A通道相位校正
        regbuf[0] = 0;
        RN8209D_WriteReg(Reg_PhsA,regbuf,1);
        //B通道相位校正
        regbuf[0] = 0;
        RN8209D_WriteReg(Reg_PhsB,regbuf,1);
        //A通道有功功率offset
        regbuf[0] = 0;
        regbuf[1] = 0;
        RN8209D_WriteReg(Reg_APOSA,regbuf,2);
        //B通道有功功率offset
        regbuf[0] = 0;
        regbuf[1] = 0;
        RN8209D_WriteReg(Reg_APOSB,regbuf,2);
        //A通道无功功率offset
        regbuf[0] = 0;
        regbuf[1] = 0;
        RN8209D_WriteReg(Reg_RPOSA,regbuf,2);
        //B通道无功功率offset
        regbuf[0] = 0;
        regbuf[1] = 0;
        RN8209D_WriteReg(Reg_RPOSB,regbuf,2);
        //A通道电流offset
        regbuf[0] = 0;
        regbuf[1] = 0;
        RN8209D_WriteReg(Reg_IARMSOS,regbuf,2);
        //B通道电流offset
        regbuf[0] = 0;
        regbuf[1] = 0;
        RN8209D_WriteReg(Reg_IBRMSOS,regbuf,2);
        //校表时不要写保护

    }
}

/********************************************************
功能描述：   误差法校准功率增益
参数说明：
返回说明：
调用方式：
全局变量：
读写时间：
注意事项：
日期    ：
********************************************************/
void RN8209D_CalibrateGPQxErr(uint8_t phase,int16_t err)
{
    const uint16_t regGPx[]={Reg_GPQA,Reg_GPQB};
    const uint16_t regArry[]={Reg_PowerPA,Reg_PowerPB};
    uint8_t regbuf[5];
    float k = 0;
    uint16_t GPQx;
    uint16_t tempValue;

    //判断是否是负数
    if(err & 0x8000){
        err &= 0x7fff;
        err = -err;
    }

    k = (-err/10000.0)/(1+err/10000.0);
    if(k > 0){
        GPQx = (uint16_t)(k*32768);
        //写使能
        regbuf[0] = 0xE5;
        RN8209D_WriteReg(WREN,regbuf,1);
        regbuf[0] = GPQx/256;
        regbuf[1] = GPQx%256;
        RN8209D_WriteReg(regGPx[phase],regbuf,2);
    }
    else{
        GPQx = (uint16_t)(k*32768+65536);
        //写使能
        regbuf[0] = 0xE5;
        RN8209D_WriteReg(WREN,regbuf,1);
        //写寄存器
        regbuf[0] = GPQx/256;
        regbuf[1] = GPQx%256;
        RN8209D_WriteReg(regGPx[phase],regbuf,2);
        m_memset(regbuf,0x00,2);
        RN8209D_ReadReg(regGPx[phase],regbuf,2);
    }
    if(phase == phase_A){
        StDef_RN8209DPara_Reg.Cst_GPQA = GPQx;
        if(RN8209D_ReadReg(regArry[phase],regbuf,2)==0){
            tempValue = regbuf[0]*256+regbuf[1];
            //StDef_RN8209DPara_Reg.Cst_Kpa = tempValue / 1100;
        }
    }
    else if(phase == phase_B){
        StDef_RN8209DPara_Reg.Cst_GPQB = GPQx;
        if(RN8209D_ReadReg(regArry[phase],regbuf,2)==0){
            tempValue = regbuf[0]*256+regbuf[1];
            //StDef_RN8209DPara_Reg.Cst_Kpb = tempValue / 1100;
        }
    }
}

/********************************************************
功能描述：   误差法校准相位
参数说明：
返回说明：
调用方式：
全局变量：
读写时间：
注意事项：
日期    ：
********************************************************/
void RN8209D_CalibratePhsXErr(uint8_t phase,int16_t err)
{
    double k = 0;
    uint8_t phsValue = 0;
    const uint16_t regGPx[]={Reg_PhsA,Reg_PhsB};
    uint8_t regbuf[5];

    k = asin(-err/10000.0/1.732)*180/3.142;
    if(k > 0){
        phsValue = (uint8_t)(k/0.02);
    }
    else{
        phsValue = (uint8_t)(k/0.02+512);
    }
    //写使能
    regbuf[0] = 0xE5;
    RN8209D_WriteReg(WREN,regbuf,1);
    //写寄存器
    regbuf[0] = phsValue;
    RN8209D_WriteReg(regGPx[phase],regbuf,1);
    if(phase == phase_A){
        StDef_RN8209DPara_Reg.Cst_PhsA = phsValue;
    }
    else if(phase == phase_B){
        StDef_RN8209DPara_Reg.Cst_PhsB = phsValue;
    }
}

/********************************************************
功能描述：   计算有功功率offset
参数说明：
返回说明：
调用方式：
全局变量：
读写时间：
注意事项：
日期    ：
********************************************************/
//void RN8209D_CalibrateAPOSx(uint8_t phase)
//{
//    uint8_t regbuf[5];
//    uint32_t regtemp[12],regtotal=0;
//    const uint16_t regArry[]={Reg_PowerPA,Reg_PowerPB};
//    const uint16_t reg_APOSArry[]={Reg_APOSA,Reg_APOSB};
//    uint8_t i = 0;
//    uint16_t temp;

//    for(i=0;i<12;i++){
//        if(RN8209D_ReadReg(regArry[phase],regbuf,4)==0){
//            regtemp[i] = (regbuf[0]<<24)+(regbuf[1]<<16)+(regbuf[2]<<8)+(regbuf[3]);
//            //求补码
//            if(regtemp[i]&0x80000000){
//               regtemp[i] = ~regtemp[i];
//               regtemp[i] += 1;
//            }
//        }
//        delay_ms(350);
//    }
//    //第一个数据不要
//    for(i=1;i<12;i++){
//        regtotal += regtemp[i];
//    }
//    regtotal /= 11;
    
//    temp = (regtotal&0x0000FFFF);
//    //写使能
//    regbuf[0] = 0xE5;
//    RN8209D_WriteReg(WREN,regbuf,1);
//    //写寄存器
//    regbuf[0] = temp/256;regbuf[1] = temp%256;
//    RN8209D_WriteReg(reg_APOSArry[phase],regbuf,2);
//}

void RN8209D_CalibrateAPOSx(uint8_t phase)
{
    uint8_t regbuf[5];
    uint32_t regtemp[12],regtotal=0;
    const uint16_t regArry[]={Reg_PowerPA,Reg_PowerPB};
    const uint16_t reg_APOSArry[]={Reg_APOSA,Reg_APOSB};
    const uint16_t reg_GPQxArry[]={Reg_GPQA,Reg_GPQB};
    uint8_t i = 0;
    uint16_t temp;
    float gGPQx = 0;
    double k = 0;

    for(i=0;i<12;i++){
        if(RN8209D_ReadReg(regArry[phase],regbuf,4)==0){
            regtemp[i] = (regbuf[0]<<24)+(regbuf[1]<<16)+(regbuf[2]<<8)+(regbuf[3]);
            //求补码
            if(regtemp[i]&0x80000000){
               regtemp[i] = ~regtemp[i];
               regtemp[i] += 1;
            }
        }
        delay_ms(350);
    }
    //第一个数据不要
    for(i=1;i<12;i++){
        regtotal += regtemp[i];
    }
    regtotal /= 11;

    RN8209D_ReadReg(reg_GPQxArry[phase],regbuf,2);
    temp = regbuf[0]*256+regbuf[1];
    if(temp&0x8000)
    {
        gGPQx = (temp-65536)/32768.0;
    }
    else{
        gGPQx = temp/32768.0;
    }

    k = (602299-regtotal)/(1+gGPQx);
    if(k > 0){
        temp = (uint16_t)k;
    }
    else{
        temp = (uint16_t)(k+65536);
    }
    //写使能
    regbuf[0] = 0xE5;
    RN8209D_WriteReg(WREN,regbuf,1);
    //写寄存器
    regbuf[0] = temp/256;regbuf[1] = temp%256;
    RN8209D_WriteReg(reg_APOSArry[phase],regbuf,2);
    if(phase == phase_A){
        StDef_RN8209DPara_Reg.Cst_APOSA = temp;
    }
    else if(phase == phase_B){
        StDef_RN8209DPara_Reg.Cst_APOSB = temp;
    }

}

/********************************************************
功能描述：   计算电流通道offset
参数说明：
返回说明：
调用方式：
全局变量：
读写时间：
注意事项：
日期    ：
********************************************************/
void RN8209D_CalibrateCurrentOffset(uint8_t phase)
{
    uint8_t regbuf[5];
    uint32_t regtemp[12],regtotal=0;
    const uint16_t regArry[]={Reg_IARMS,Reg_IBRMS};
    const uint16_t regIx_OS[]={Reg_IARMSOS,Reg_IBRMSOS};
    uint8_t i = 0;
    uint16_t temp;

    for(i=0;i<12;i++)
    {
        if(RN8209D_ReadReg(regArry[phase],regbuf,3)==0){
            regtemp[i] = (regbuf[0]<<16)+(regbuf[1]<<8)+(regbuf[2]);
        }       
        delay_ms(350);
    }
    //第一个数据不要
    for(i=1;i<12;i++){
        regtotal += regtemp[i];
    }
    regtotal /= 11;
    regtotal = regtotal * regtotal;
    //求反码
    regtotal = ~regtotal;
    temp = (regtotal>>8);
    //符号位
    if(regtotal & 0x80000000)
        temp |= 0x8000;
    //写使能
    regbuf[0] = 0xE5;
    RN8209D_WriteReg(WREN,regbuf,1);
    //写寄存器
    regbuf[0] = temp/256;regbuf[1] = temp%256;
    RN8209D_WriteReg(regIx_OS[phase],regbuf,2);
    if(phase == phase_A){
        StDef_RN8209DPara_Reg.Cst_IARMSOS = temp;
    }
    else if(phase == phase_B){
        StDef_RN8209DPara_Reg.Cst_IBRMSOS = temp;
    }
}

/********************************************************
功能描述：   计算电压、电流显示转换系数
参数说明：
返回说明：
调用方式：
全局变量：
读写时间：
注意事项：
日期    ：
********************************************************/
void RN8209D_CalibrateKx(uint8_t phase)
{
    uint8_t regbuf[3];
    //uint32_t tempValue;
    uint32_t regtemp[12],regtotal=0;
    uint8_t i = 0;

    if(phase == phase_A){
        for(i=0;i<12;i++)
        {
            if(RN8209D_ReadReg(Reg_URMS,regbuf,3)==0){
                regtemp[i] = (regbuf[0]<<16)+(regbuf[1]<<8)+(regbuf[2]);
            }
            delay_ms(350);
        }
        //第一个数据不要
        for(i=1;i<12;i++){
            regtotal += regtemp[i];
        }
        regtotal /= 11;
        StDef_RN8209DPara_Reg.Cst_Ku = regtotal / 220;

        regtotal = 0;

        for(i=0;i<12;i++)
        {
            if(RN8209D_ReadReg(Reg_IARMS,regbuf,3)==0){
                regtemp[i] = (regbuf[0]<<16)+(regbuf[1]<<8)+(regbuf[2]);
            }
            delay_ms(350);
        }
        //第一个数据不要
        for(i=1;i<12;i++){
            regtotal += regtemp[i];
        }
        regtotal /= 11;
        StDef_RN8209DPara_Reg.Cst_Kia = regtotal / 5;
    }
    else if(phase == phase_B){
        for(i=0;i<12;i++)
        {
            if(RN8209D_ReadReg(Reg_URMS,regbuf,3)==0){
                regtemp[i] = (regbuf[0]<<16)+(regbuf[1]<<8)+(regbuf[2]);
            }
            delay_ms(350);
        }
        //第一个数据不要
        for(i=1;i<12;i++){
            regtotal += regtemp[i];
        }
        regtotal /= 11;
        StDef_RN8209DPara_Reg.Cst_Ku = regtotal / 220;

        regtotal = 0;

        for(i=0;i<12;i++)
        {
            if(RN8209D_ReadReg(Reg_IBRMS,regbuf,3)==0){
                regtemp[i] = (regbuf[0]<<16)+(regbuf[1]<<8)+(regbuf[2]);
            }
            delay_ms(350);
        }
        //第一个数据不要
        for(i=1;i<12;i++){
            regtotal += regtemp[i];
        }
        regtotal /= 11;
        StDef_RN8209DPara_Reg.Cst_Kib = regtotal / 5;
    }
}


/********************************************************
功能描述：
参数说明：
返回说明： 扩大10倍
调用方式：
全局变量：
读写时间：
注意事项：
日期    ：
********************************************************/
uint8_t RN8209D_ReadVoltage(uint16_t *vol)
{
    uint8_t  regbuf[3];
    uint32_t tempValue;

    if(RN8209D_ReadReg(Reg_URMS,regbuf,3)==0){
        tempValue = (regbuf[0]<<16)+(regbuf[1]<<8)+(regbuf[2]);
        if(tempValue & 0x800000){
            tempValue = 0;
        }
        else{
            *vol = (uint16_t)(tempValue*10.0/StDef_RN8209DPara_Reg.Cst_Ku);
        }
        return 0;
    }
    
    return 1;
    
}

/********************************************************
功能描述：
参数说明：
返回说明： 扩大1000倍
调用方式：
全局变量：
读写时间：
注意事项：
日期    ：
********************************************************/
uint8_t RN8209D_ReadCurrent(uint8_t phase,uint16_t *current)
{
    uint8_t  regbuf[3];
    uint32_t tempValue;

    if(phase == phase_A){
        if(RN8209D_ReadReg(Reg_IARMS,regbuf,3)==0){
            tempValue = (regbuf[0]<<16)+(regbuf[1]<<8)+(regbuf[2]);
            if(tempValue & 0x800000){
                tempValue = 0;
            }
            else{
                *current = (uint16_t)(tempValue*1000.0/StDef_RN8209DPara_Reg.Cst_Kia);
            }
            return 0;
        }
    }
    else if(phase == phase_B){
        if(RN8209D_ReadReg(Reg_IBRMS,regbuf,3)==0){
            tempValue = (regbuf[0]<<16)+(regbuf[1]<<8)+(regbuf[2]);
            if(tempValue & 0x800000){
                tempValue = 0;
            }
            else{
                *current = (uint16_t)(tempValue*1000.0/StDef_RN8209DPara_Reg.Cst_Kib);
            }
            return 0;
        }
    }
    return 1;
}

/********************************************************
功能描述：   读取有功功率
参数说明：
返回说明：   扩大1000倍
调用方式：
全局变量：
读写时间：
注意事项：
日期    ：
********************************************************/
uint8_t RN8209D_ReadPower(uint8_t phase,uint32_t *p)
{
    uint8_t  regbuf[4];
    uint32_t tempValue;

    if(phase == phase_A){
        if(RN8209D_ReadReg(Reg_PowerPA,regbuf,4)==0){
            tempValue = (regbuf[0]<<24)+(regbuf[1]<<16)+(regbuf[2]<<8)+(regbuf[3]);
            if(tempValue&0x80000000){
              tempValue = ~tempValue;
              tempValue += 1;
            }
//            if(tempValue>=21903393){
//                *p = (uint32_t)(tempValue*0.0000457);
//            }
//            else if(tempValue<21903393 && tempValue>=2190339){
//                *p = (uint32_t)(tempValue*10*0.0000457);
//            }
//            else if(tempValue<2190339 && tempValue>=219033){
//                *p = (uint32_t)(tempValue*100*0.0000457);
//            }
//            else if(tempValue<219033){
//                *p = (uint32_t)(tempValue*1000*0.0000457);
//            }

            *p = (uint32_t)(tempValue*0.000457);
            return 0;
        }
    }
    else if(phase == phase_B){
        if(RN8209D_ReadReg(Reg_PowerPB,regbuf,4)==0){
            tempValue = (regbuf[0]<<24)+(regbuf[1]<<16)+(regbuf[2]<<8)+(regbuf[3]);
            if(tempValue&0x80000000){
              tempValue = ~tempValue;
              tempValue += 1;
            }
            *p = (uint32_t)(tempValue*0.0000457);
            return 0;
        }
    }
    return 1;
}

/********************************************************
功能描述：   读取累计电量
参数说明：
返回说明：   扩大100倍
调用方式：
全局变量：
读写时间：
注意事项：
日期    ：
********************************************************/
uint8_t RN8209D_ReadTotalE(uint8_t phase,uint32_t *p)
{
    uint8_t  regbuf[3];
    uint32_t tempValue;

    if(phase == phase_A){
        if(RN8209D_ReadReg(Reg_EnergyP,regbuf,3)==0){
            tempValue = (regbuf[0]<<16)+(regbuf[1]<<8)+(regbuf[2]);
            //*p = (uint32_t)(tempValue*100.0/3200);
            *p = tempValue;
            return 0;
        }
    }
    else if(phase == phase_B){
        if(RN8209D_ReadReg(Reg_EnergyD,regbuf,3)==0){
            tempValue = (regbuf[0]<<16)+(regbuf[1]<<8)+(regbuf[2]);
            //*p = (uint32_t)(tempValue*100.0/3200);
            *p = tempValue;
            return 0;
        }
    }
    return 1;
}

/********************************************************
功能描述：   读取校验寄存器
参数说明：
返回说明：
调用方式：
全局变量：
读写时间：
注意事项：
日期    ：
********************************************************/
uint8_t RN8209D_ReadCheckSum(uint32_t *p)
{
    uint8_t  regbuf[3];
    uint32_t tempValue;

    if(RN8209D_ReadReg(Reg_EMUStatus,regbuf,3)==0){
        tempValue = (regbuf[0]<<16)+(regbuf[1]<<8)+(regbuf[2]);
        *p = tempValue;
        return 0;
    }
    return 1;
}

