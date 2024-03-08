/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LORA__H
#define __LORA__H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
extern char RFlag;


void LoRa_Config(void);
void LoRa_SEND(const char* data);


#ifdef __cplusplus
}
#endif

#endif /*__LORA__H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
