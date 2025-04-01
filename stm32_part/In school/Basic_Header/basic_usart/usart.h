#include "stm32f10x.h"
uint8_t uart2Read(void);
uint16_t uart2Available(void);
void USART2_IRQHandler(void);
void USART2_Send_Data(u8 *buf,u8 len);
void USART2_Init(u32 bound);
void TR_Receive(void);

