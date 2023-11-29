#include "max31865_1.h"
#include <math.h>
#include "spi1.h"

	
//初始化SPI max31865 的IO口
void SPI1_MAX31865_Init(void)
{
	SPI1_Init();		   		                //初始化SPI1
	SPI1_SetSpeed(SPI_BaudRatePrescaler_64);	//设置为 72/64 = 1.125 M时钟
	writeRegister8_1(0x00, 0x00);                 //清除配置寄存器
	enableBias_1(1);                              //使能偏置电压
	delay_ms(10);                               //等待10ms使得RTDIN的滤波电容充电
	setWires_1(MAX31865_3WIRE);                   //使能PT100 三线工作模式
	clearFault_1();                               //清除故障检测位
	
}  

//RTD接线模式设置


void setWires_1(max31865_numwires_t wires)
{
  uint8_t t = readRegister8_1(MAX31856_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    t |= MAX31856_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    t &= ~MAX31856_CONFIG_3WIRE;
  }
  writeRegister8_1(MAX31856_CONFIG_REG, t);
}

//设置自动转换模式
void autoConvert_1(bool b)  
{
  uint8_t t = readRegister8_1(MAX31856_CONFIG_REG);
  if (b) {
   	  t |= MAX31856_CONFIG_MODEAUTO;       // enable autoconvert
  } else {
    t &= ~MAX31856_CONFIG_MODEAUTO;       // disable autoconvert
  }
  writeRegister8_1(MAX31856_CONFIG_REG, t);
}

 //使能偏执电压
void enableBias_1(bool b) 
{
  uint8_t t = readRegister8_1(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_BIAS;       // enable bias
  } else {
    t &= ~MAX31856_CONFIG_BIAS;       // disable bias
  }
  writeRegister8_1(MAX31856_CONFIG_REG, t);
}	

//读寄存器，8位
u8  readRegister8_1(u8 addr) 
{
  uint8_t ret = 0;
  addr &= 0x7F;                      // make sure top bit is set to 1
  SPI1_FLASH_CS=0; 
  delay_us(100);
  SPI1_ReadWriteByte(addr);  //发送写取状态寄存器命令    
  ret = SPI1_ReadWriteByte(0xff);   //写入一个字节  
  delay_us(100);
  SPI1_FLASH_CS=1;                   //取消片选      
  return ret;
}

//读两个寄存器，16位
u16 readRegister16_1(u8 addr)  
{
  uint8_t buffer[2] = {0, 0};
  uint16_t ret = 0;
  readRegisterN_1(addr, buffer, 2);

  ret = buffer[0];
  ret <<= 8;
  ret |=  buffer[1];
  
  return ret;
}

 //读N个寄存器
void readRegisterN_1(u8 addr, u8 buffer[], u8 n)         
{
  addr &= 0x7F;                             // make sure top bit is 0
  SPI1_FLASH_CS=0; 
  delay_us(100);
  SPI1_ReadWriteByte(addr);                  //发送读取状态寄存器命令    
  while (n--) {
    buffer[0] = SPI1_ReadWriteByte(0Xff);
    buffer++;
  }
  delay_us(100);
  SPI1_FLASH_CS=1;                            //取消片选  
}

//指定寄存器写8位数据
void  writeRegister8_1(u8 addr, u8 data)  
{ 
  addr |= 0x80;                              // make sure top bit is set to 1
  SPI1_FLASH_CS=0;
  delay_us(100);
  SPI1_ReadWriteByte(addr);                  //发送读取状态寄存器命令    
  SPI1_ReadWriteByte(data);                  //写入一个字节 
  delay_us(100);
  SPI1_FLASH_CS=1;                            //取消片选                 
}	

//RTD数据读取
u16 readRTD_1()             
{
  uint8_t t = 0;
  uint16_t rtd = 0;
  t = readRegister8_1(MAX31856_CONFIG_REG);
  t |= MAX31856_CONFIG_1SHOT;      
  writeRegister8_1(MAX31856_CONFIG_REG, t);
  delay_ms(70);                                   //单次转换读取时间在片选信号拉高后在50HZ工作模式下需要约62ms，60hz约52ms
  rtd = readRegister16_1(MAX31856_RTDMSB_REG);
  // remove fault
  rtd >>= 1;

  return rtd;
}

//温度计算
float temperature_1(float RTDnominal, float refResistor) 
{
  float Z1, Z2, Z3, Z4, Rt, temp;
  float rpoly;
  Rt = readRTD_1();
  Rt /= 32768;
  Rt *= refResistor;

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;
  
  if (temp >= 0) return temp;

  // ugh.
  rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt;  // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt;  // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt;  // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt;  // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}	

 //故障检测
u8 readFault_1(void)  
{
  return readRegister8_1(MAX31856_FAULTSTAT_REG);
}	

//清除故障标志位
void clearFault_1(void)
{
  uint8_t t = readRegister8_1(MAX31856_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31856_CONFIG_FAULTSTAT;
  writeRegister8_1(MAX31856_CONFIG_REG, t);
}
	




//SPI测试函数
u8  writetest_1(u8 n)  
{ 
  u8 ret;
  SPI1_FLASH_CS=0; 
  ret = SPI1_ReadWriteByte(n);               //发送读取状态寄存器命令       
  SPI1_FLASH_CS=1;                            //取消片选    
  return ret;	
}	
















