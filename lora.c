#include "lora.h"
#include "main.h"
#include "at.h"
#include "iwdg.h"


char RFlag = 0;
char LoRa_RST_FLAG = 0;
uint8_t SENDN = 0;


void LoRa_SEND(const char* data){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	SENDN = 0xAA;
	HAL_UART_Transmit(&hlpuart1, &SENDN, 1, HAL_MAX_DELAY);
	lpu1_print(data);
	SENDN = 0xFF;
	HAL_UART_Transmit(&hlpuart1, &SENDN, 1, HAL_MAX_DELAY);
	HAL_Delay(15);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

static void LoRa_SEND_ID(void){
	lpu1_print("ID:%08X%08X\n", uid_array[1], uid_array[2]);
	HAL_Delay(15);
}

static void LoRa_SEND_VER(void){
	lpu1_print("VER:%s\n", VER);
	HAL_Delay(15);
}

static void LoRa_SEND_ALL(void){
	lpu1_print("ID:%08X%08X\n",uid_array[1], uid_array[2]);
	lpu1_print("VER:%s\n", VER);
	lpu1_print("LoRa_CH:%d\nLoRa_PID:%d\nLoRa_PWR:%d\nLoRa_BR:%d\nSY_WKT:%d\nSY_UPT:%d\nSY_TH:%d\nSY_LED:%d\n"
	,config_data.LoRa_CH, config_data.LoRa_PID, config_data.LoRa_PWR, config_data.LoRa_BR, config_data.SY_WKT, config_data.SY_UPT, config_data.SY_TH, config_data.SY_LED);
	HAL_Delay(100);
}

static void LoRa_CMD(const char* at){
	lpu1_print("%s\r\n",at);
	u2_print("%s\r\n",at);  
	HAL_Delay(100);
}

static void LoRa_SET_MODE(int n)
{
	if(n == 0)LoRa_CMD("AT+MODE=0");
	else if(n == 1)LoRa_CMD("AT+MODE=1");
}

static void LoRa_SET_LPWR(int n)
{
	if(n == 0)LoRa_CMD("AT+LPWR=0");
	else if(n == 1)LoRa_CMD("AT+LPWR=1");
}

static void LoRa_SET_BAUD(int baud)
{
	char temp[32];
	sprintf(temp,"AT+UART=%d,0,0",baud);
	LoRa_CMD(temp);
}


static void LoRa_SET_BR(int brn)
{
	char temp[32];
	switch(brn){
		case 1:
			sprintf(temp,"AT+RFBR=1.11K");
			break;
		case 2:
			sprintf(temp,"AT+RFBR=1.46K");
			break;
		case 3:
			sprintf(temp,"AT+RFBR=2.6K");
			break;
		case 4:
			sprintf(temp,"AT+RFBR=4.56K");
			break;
		case 5:
			sprintf(temp,"AT+RFBR=9.11K");
			break;
		case 6:
			sprintf(temp,"AT+RFBR=18.23K");
			break;
	}
	LoRa_CMD(temp);
}


static void LoRa_SET_CH(int ch)
{
	char temp[32];
	sprintf(temp,"AT+RFCH=%d",ch);
	LoRa_CMD(temp);
}

static void LoRa_SET_PID(int pid)
{
	char temp[32];
	sprintf(temp,"AT+PID=%d",pid);
	LoRa_CMD(temp);
}

static void LoRa_SET_PWR(int pwr)
{
	char temp[32];
	sprintf(temp,"AT+PWR=%d",pwr);
	LoRa_CMD(temp);
}

static void LoRa_RESET(void)
{
	char temp[32];
	sprintf(temp,"AT+RST");
	LoRa_CMD(temp);
}


static void LoRa_Go_Config(void)
{
	LoRa_SET_MODE(0); //退出透传
	LoRa_SET_CH(0);
	LoRa_SET_PID(255);
	LoRa_SET_BR(5);
	LoRa_SET_MODE(1); //进入透传
}

static void LoRa_Out_Config(void)
{
	LoRa_SET_MODE(0); //退出透传
	LoRa_SET_CH(config_data.LoRa_CH);
	LoRa_SET_PID(config_data.LoRa_PID);
	LoRa_SET_PWR(config_data.LoRa_PWR);
	LoRa_SET_BR(config_data.LoRa_BR);
	LoRa_SET_LPWR(1); //启动低功耗模式
	if(LoRa_RST_FLAG == 1)
		LoRa_RESET();
	LoRa_SET_MODE(1); //进入透传
}


void LoRa_Config(void)
{
	char temp[64] = {0};
	int ia = 50;
	int num = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	LoRa_Go_Config();
	while(ia--){
		HAL_IWDG_Refresh(&hiwdg);
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(50);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_Delay(50);
		if(RFlag) break;
	}
	if(ia>0){
		LoRa_SEND_ID();
		LoRa_SEND_VER();
		ia = 300;
		while(ia--){
			HAL_IWDG_Refresh(&hiwdg);
			
			if(RXEnd == 1 && RXBuff1[0] == 0x41){
				RXEnd = 0;
				ia = 300;
				memcpy(temp,&RXBuff1,strlen(RXBuff1));
			  u2_print(temp);
				num = parse_AT_command(temp);
				memset(temp,0,sizeof(temp));
				memset(RXBuff1,0,sizeof(RXBuff1));
				RXBuff1_Index = 0;
				u2_print("AT_BACK:%d\n", num);
				if(num == 0){
					u2_print("AT_OK\n");
				}else if(num == 1){
					LoRa_SEND_ID();
				}else if(num == 2){
					LoRa_SEND_ALL();
				}else if(num == 9){
					LoRa_RST_FLAG = 1;
				}else if(num == 10){
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(1000);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_Delay(1000);
					break;
				}
			}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_Delay(100);
			
		}
		Save_Config_to_Flash();
	}
	LoRa_Out_Config();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}