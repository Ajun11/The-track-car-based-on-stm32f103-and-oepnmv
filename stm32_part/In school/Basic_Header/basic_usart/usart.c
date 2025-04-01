#include "sys.h"		    
#include "delay.h"
#include "usart.h"
#include "stdio.h"
#define UART2_BUFFER_SIZE 256 
volatile uint8_t rx2Buffer[UART2_BUFFER_SIZE]; //可以容纳256个字节的缓冲数组
volatile uint8_t rxBufferNum  = 0;
volatile uint8_t h_u32RecCnt = 0; 
static volatile uint8_t rx2BufferRTail  = 0;
//u8 str1[8];
u8 str_get[7];
short  x_axis=0;
short  y_axis=0;
int blob_flag=-1;
//初始化IO 串口2
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率	  
void USART2_Init(u32 bound)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOA,时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟 
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	//PA2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//Tx
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入Rx
  GPIO_Init(GPIOA, &GPIO_InitStructure);  

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//复位串口2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//停止复位
 
	 
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;///奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式

  USART_Init(USART2, &USART_InitStructure); ; //初始化串口
  
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //使能串口2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //从优先级2级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
 
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
   
  USART_Cmd(USART2, ENABLE);                    //使能串口 
   
}

//发送len个字节.
//buf:发送区首地址
//len:发送的字节数
void USART2_Send_Data(u8 *buf,u8 len)
{
	u8 t; 
  for(t=0;t<len;t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
	}	 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);  
}
  



void TR_Receive(void)
{
	unsigned char BufC; 
	if(uart2Available())//判断是否触发了终断
	{
		BufC = uart2Read(); //读一个字节
		if((BufC==0xFF))//判断是否为帧 头
		{ 
			BufC = uart2Read(); //读一个字节
			if((BufC==0xF1))//是否为第二个帧头，因为openmv一般传输两位
			{
				str_get[2]=uart2Read();
				str_get[3]=uart2Read();
				str_get[4]=uart2Read();
				str_get[5]=uart2Read();
				str_get[6]=uart2Read();
				blob_flag=str_get[2];
				x_axis=(short)(str_get[3]+(str_get[4]<<8));//openmv存储数据的方式是小端模式，即数据的高字节存储到低地址，低字节存储到高地址
				y_axis=(short)(str_get[5]+(str_get[6]<<8));//例如串口接收数据0xff01,那么先接收到的是0x01,后接到0xff，这里为什么高字节左移
				//因为传进来的是个int类型的数字，可能是负数，那么就对高字节左移八位加上低字节即可。
			}
			 
		}
	} 
}



/**
* @brief      UartRead
* @param[out]  Ret 1 Byte Data.
* @param[in]  void.
*/
uint8_t uart2Read(void)
{
    uint8_t ch ;
    while  (!h_u32RecCnt);//非0时执行下面语句，所以说，h_u32RecCnt=1时执行读取的语句

    ch = rx2Buffer[rx2BufferRTail];
    rx2BufferRTail++;
    if (rx2BufferRTail>=UART2_BUFFER_SIZE)
    {
        rx2BufferRTail  = 0;
    } 
    h_u32RecCnt--;//这里就变成了0，只有当再次触发中断才会变成1
    return ch;
}

uint16_t uart2Available(void)
{
    return h_u32RecCnt;
}

void USART2_IRQHandler(void)
{ //千万不能用USART_GetITStatus
	if(USART_GetFlagStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断有效,若接收数据寄存器满
	{
		rx2Buffer[rxBufferNum] = USART_ReceiveData(USART2); //接收8 Bit数据 就是一字节数据,例如0x03
		rxBufferNum++;
		if (rxBufferNum >= UART2_BUFFER_SIZE ) rxBufferNum = 0;//这里的判断仅仅是设置存储的限制而已，即此数组可以一共存储256字节数据
		h_u32RecCnt++;//标志进数据的标志位
  } 
} 
//这个应该是我写的
uint8_t usart_read_byte(void)
{
	uint8_t values;
	if(h_u32RecCnt!=0)//说明有东西可读
	{
		values=rx2Buffer[rx2BufferRTail];
		rx2BufferRTail++;
	}
	if(rx2BufferRTail==256)
		rx2BufferRTail=0;
  h_u32RecCnt=0;
	return values;
}









