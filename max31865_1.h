/*
	作者：Mr.luo
	创建时间：2018.11.28
	修改时间：2018.11.28
	内容：max31865 驱动头文件
*/

#ifndef _MAX31865_H
#define _MAX31865_H

#include "sys.h" 
#include "spi1.h"

#include "stdio.h"
#include "delay.h"

//STM32相关端口和库函数宏定义
#define	SPI1_FLASH_CS             PAout(4)  	//PA4用来生成片选信号
#define SPI_ReadWriteByte        SPI1_ReadWriteByte


#define MAX31856_CONFIG_REG            0x00
#define MAX31856_CONFIG_BIAS           0x80
#define MAX31856_CONFIG_MODEAUTO       0x40
#define MAX31856_CONFIG_MODEOFF        0x00
#define MAX31856_CONFIG_1SHOT          0x20
#define MAX31856_CONFIG_3WIRE          0x10
#define MAX31856_CONFIG_24WIRE         0x00
#define MAX31856_CONFIG_FAULTSTAT      0x02
#define MAX31856_CONFIG_FILT50HZ       0x01
#define MAX31856_CONFIG_FILT60HZ       0x00

#define MAX31856_RTDMSB_REG           0x01
#define MAX31856_RTDLSB_REG           0x02
#define MAX31856_HFAULTMSB_REG        0x03
#define MAX31856_HFAULTLSB_REG        0x04
#define MAX31856_LFAULTMSB_REG        0x05
#define MAX31856_LFAULTLSB_REG        0x06
#define MAX31856_FAULTSTAT_REG        0x07


#define MAX31865_FAULT_HIGHTHRESH     0x80
#define MAX31865_FAULT_LOWTHRESH      0x40
#define MAX31865_FAULT_REFINLOW       0x20
#define MAX31865_FAULT_REFINHIGH      0x10
#define MAX31865_FAULT_RTDINLOW       0x08
#define MAX31865_FAULT_OVUV           0x04


#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7


typedef enum max31865_numwires { 
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;            //接线模式枚举定义

void SPI1_MAX31865_Init(void);

void setWires_1(max31865_numwires_t wires);  //RTD接线模式设置
void autoConvert_1(bool b);  //设置自动转换模式
void enableBias_1(bool b);   //使能偏执电压
u8  writetest_1(u8 n);       //SPI测试函数

u8  readRegister8_1(u8 addr);    //读寄存器，8位
u16 readRegister16_1(u8 addr);   //读两个寄存器，16位
void readRegisterN_1(u8 addr, u8 buffer[], u8 n);          //读N个寄存器

void     writeRegister8_1(u8 addr, u8 data);                //指定寄存器写8位数据

u16 readRTD_1(void);             //RTD数据读取
float temperature_1(float RTDnominal, float refResistor);  //温度计算

u8 readFault_1(void);        //故障检测
void clearFault_1(void);     //清除故障标志位
u8 MAX31865_1(u8 spi1_cs, u8 spi1_mosi, u8 spi1_miso, u8 spi1_clk);

#endif
