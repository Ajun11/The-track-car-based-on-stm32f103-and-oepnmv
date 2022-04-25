/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp_led.h"
#include "usart.h"
void haodelay(uint32_t time)//10ms
{
	for(int i=0;i<1000;i++)
	{
		for(int j=0;j<72;j++)
		{
			for(int k=0;k<time;k++);
		}
	}
	
}
//#include "basictime.h"

//extern volatile uint32_t time;


/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault sexception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
//void  BASIC_TIM_IRQHandler (void)
//{
//	if ( TIM_GetITStatus( BASIC_TIM, TIM_IT_Update) != RESET ) 
//	{	
//		time++;
//		TIM_ClearITPendingBit(BASIC_TIM , TIM_FLAG_Update);  		 
//	}		 	
//}
//void GENERAL_TIM_INT_FUN(void)//这是超声波测距的中断函数
//{
//	// 当要被捕获的信号的周期大于定时器的最长定时时，定时器就会溢出，产生更新中断
//	// 这个时候我们需要把这个最长的定时周期加到捕获信号的时间里面去
//	if ( TIM_GetITStatus ( GENERAL_TIM, TIM_IT_Update) != RESET )               
//	{	
//		TIM_ICUserValueStructure.Capture_Period ++;		
//		TIM_ClearITPendingBit ( GENERAL_TIM, TIM_FLAG_Update ); 		
//	}

//	// 上升沿捕获中断
//	if ( TIM_GetITStatus (GENERAL_TIM, GENERAL_TIM_IT_CCx ) != RESET)
//	{
//		// 第一次捕获
//		if ( TIM_ICUserValueStructure.Capture_StartFlag == 0 )
//		{
//			// 计数器清0
//			TIM_SetCounter ( GENERAL_TIM, 0 );
//			// 自动重装载寄存器更新标志清0
//			TIM_ICUserValueStructure.Capture_Period = 0;
//      // 存捕获比较寄存器的值的变量的值清0			
//			TIM_ICUserValueStructure.Capture_CcrValue = 0;

//			// 当第一次捕获到上升沿之后，就把捕获边沿配置为下降沿
//			GENERAL_TIM_OCxPolarityConfig_FUN(GENERAL_TIM, TIM_ICPolarity_Falling);
//      // 开始捕获标准置1			
//			TIM_ICUserValueStructure.Capture_StartFlag = 1;			
//		}
//		// 下降沿捕获中断
//		else // 第二次捕获
//		{
//			// 获取捕获比较寄存器的值，这个值就是捕获到的高电平的时间的值
//			TIM_ICUserValueStructure.Capture_CcrValue = 	GENERAL_TIM_GetCapturex_FUN (GENERAL_TIM);

//			// 当第二次捕获到下降沿之后，就把捕获边沿配置为上升沿，好开启新的一轮捕获
//			GENERAL_TIM_OCxPolarityConfig_FUN(GENERAL_TIM, TIM_ICPolarity_Rising);
//      // 开始捕获标志清0		
//			TIM_ICUserValueStructure.Capture_StartFlag = 0;
//      // 捕获完成标志置1			
//			TIM_ICUserValueStructure.Capture_FinishFlag = 1;		
//		}

//		TIM_ClearITPendingBit (GENERAL_TIM,GENERAL_TIM_IT_CCx);	    
//	}		
//}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
