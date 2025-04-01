#include "bsp_motor.h"
static void wheel_GPIO_init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		
		RCC_APB2PeriphClockCmd(left_front_wheel_clk, ENABLE);
		GPIO_InitStructure.GPIO_Pin =  left_front_wheel_pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(left_front_wheel_port, &GPIO_InitStructure);
		
		RCC_APB2PeriphClockCmd(right_front_wheel_clk, ENABLE);
		GPIO_InitStructure.GPIO_Pin =  right_front_wheel_pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(right_front_wheel_port, &GPIO_InitStructure);
		
		RCC_APB2PeriphClockCmd(left_back_wheel_clk, ENABLE);
		GPIO_InitStructure.GPIO_Pin =  left_back_wheel_pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(left_back_wheel_port, &GPIO_InitStructure);
		
		RCC_APB2PeriphClockCmd(right_back_wheel_clk, ENABLE);
		GPIO_InitStructure.GPIO_Pin =  right_back_wheel_pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(right_back_wheel_port, &GPIO_InitStructure);
		//下面是极性
	/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure_1;
		/*开启LED相关的GPIO外设时钟*/
		RCC_APB2PeriphClockCmd( left_front_wheel_control_1_clk, ENABLE);
		/*选择要控制的GPIO引脚*/
		GPIO_InitStructure_1.GPIO_Pin = left_front_wheel_control_1_pin;	

		/*设置引脚模式为通用推挽输出*/
		GPIO_InitStructure_1.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure_1.GPIO_Speed = GPIO_Speed_50MHz; 

		/*调用库函数，初始化GPIO*/
		GPIO_Init(left_front_wheel_control_1_port, &GPIO_InitStructure_1);	
	
	//**************************这是B的
		GPIO_InitTypeDef GPIO_InitStructure_1_1;

		RCC_APB2PeriphClockCmd( left_front_wheel_control_2_clk, ENABLE);

		GPIO_InitStructure_1_1.GPIO_Pin = left_front_wheel_control_2_pin|right_front_wheel_control_1_pin|right_front_wheel_control_2_pin|
		                                left_back_wheel_control_1_pin|left_back_wheel_control_2_pin|right_back_wheel_control_1_pin|
																		right_back_wheel_control_2_pin;	

		GPIO_InitStructure_1_1.GPIO_Mode = GPIO_Mode_Out_PP;   


		GPIO_InitStructure_1_1.GPIO_Speed = GPIO_Speed_50MHz; 

	
		GPIO_Init(left_front_wheel_control_2_port, &GPIO_InitStructure_1_1);	
		left_front_wheel_control_1_low;
		left_front_wheel_control_2_low;
		right_front_wheel_control_1_low;
		right_front_wheel_control_2_low;
		left_back_wheel_control_1_low;
		left_back_wheel_control_2_low;
		right_back_wheel_control_1_low;
		right_back_wheel_control_2_low;
}



//在使用电机函数之前必须在主函数中初始化
/*****************pwm******************/
void  TIM_INIT(void)
{

			ADVANCE_TIM_APBxClock_FUN_1(ADVANCE_TIM_CLK_1,ENABLE);
		/*--------------------时基结构体初始化-------------------------*/
			TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
			// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
			TIM_TimeBaseStructure.TIM_Period=TIM_PERIOD;	
			// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
			TIM_TimeBaseStructure.TIM_Prescaler= TIM_PSC;		
			// 计数器计数模式，设置为向上计数
			TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
			// 重复计数器的值，没用到不用管
			TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
			// 初始化定时器
			TIM_TimeBaseInit(ADVANCE_TIM_1, &TIM_TimeBaseStructure);
	
			// 占空比配置初始
			uint16_t CCR1_Val = 100;
			uint16_t CCR2_Val = 100;
			uint16_t CCR3_Val = 100;
			uint16_t CCR4_Val = 100;
			
			TIM_OCInitTypeDef  TIM_OCInitStructure;
			// 配置为PWM模式1
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			// 输出使能
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			// 设置占空比大小
			//TIM_OCInitStructure_1.TIM_Pulse = 7;
			// 输出通道电平极性配置
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			// 输出通道空闲电平极性配置
			//TIM_OCInitStructure_1.TIM_OCIdleState = TIM_OCIdleState_Set;
			//通道1
			TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
			TIM_OC1Init(ADVANCE_TIM_1, &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(ADVANCE_TIM_1, TIM_OCPreload_Enable);
			//通道2
			TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
			TIM_OC2Init(ADVANCE_TIM_1, &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(ADVANCE_TIM_1, TIM_OCPreload_Enable);
			//通道3
			TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
			TIM_OC3Init(ADVANCE_TIM_1, &TIM_OCInitStructure);
			TIM_OC3PreloadConfig(ADVANCE_TIM_1, TIM_OCPreload_Enable);
			//通道4
			TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
			TIM_OC4Init(ADVANCE_TIM_1, &TIM_OCInitStructure);
			TIM_OC4PreloadConfig(ADVANCE_TIM_1, TIM_OCPreload_Enable);
			// 使能计数器
			TIM_Cmd(ADVANCE_TIM_1, ENABLE);	
			// 主输出使能，当使用的是通用定时器时，这句不需要
			TIM_CtrlPWMOutputs(ADVANCE_TIM_1, ENABLE);	
}
/***************初始化函数*************/
void MOTOR_INIT(void)
{
	wheel_GPIO_init();
	TIM_INIT();
}

//左前轮前转后转以及速度
void  front_left_wheel_control(int judge_value,unsigned int Space)
{
	if(judge_value==0)//电机反转
	{
		left_front_wheel_control_1_low;
		left_front_wheel_control_2_high;
	}
	else if(judge_value==1) //电机正转
	{
		left_front_wheel_control_1_high;
		left_front_wheel_control_2_low;
	}
	TIM_SetCompare1(TIM1 ,Space);
}

//右前轮前转后转以及速度
void  front_right_wheel_control(int judge_value,unsigned int Space)
{
	if(judge_value==0)//电机反转
	{
		right_front_wheel_control_1_low;
		right_front_wheel_control_2_high;
	}
	else if(judge_value==1) //电机正转
	{
		right_front_wheel_control_1_high;
		right_front_wheel_control_2_low;
	}
	
	TIM_SetCompare2(TIM1 ,Space);
}
//左后轮前转后转以及速度
void  back_left_wheel_control(int judge_value,unsigned int Space)
{
	if(judge_value==0)//电机反转
	{
		left_back_wheel_control_1_low;
		left_back_wheel_control_2_high;
	}
	else  if(judge_value==1) //电机正转
	{
		left_back_wheel_control_1_high;
		left_back_wheel_control_2_low;
	}
	TIM_SetCompare3(TIM1 ,Space);
}
//右后轮前转后转以及速度
void  back_right_wheel_control(int judge_value,unsigned int Space)
{
	if(judge_value==0)//电机反转
	{
		right_back_wheel_control_1_low;
		right_back_wheel_control_2_high;
	}
	else  if(judge_value==1) //电机正转
	{
		right_back_wheel_control_1_high;
		right_back_wheel_control_2_low;
	}
	TIM_SetCompare4(TIM1 ,Space);
}
////前进
//void  straight_move(unsigned int space)
//{
//	front_left_wheel_control(1,space);
//	front_right_wheel_control(1,space);
//	back_left_wheel_control(1,space);
//	back_right_wheel_control(1,space);
//}
////后退
//void  straight_reverse(unsigned int space)
//{
//	front_left_wheel_control(0,space);
//	front_right_wheel_control(0,space);
//	back_left_wheel_control(0,space);
//  back_right_wheel_control(0,space);
//}
////右平移
//void  right_move(unsigned  int space)
//{
//	front_left_wheel_control(1,space);
//	front_right_wheel_control(0,space);
//	back_left_wheel_control(0,space);
//  back_right_wheel_control(1,space);
//}
////左平移
//void  left_move(unsigned int space)
//{
//	front_left_wheel_control(0,space);
//	front_right_wheel_control(1,space);
//	back_left_wheel_control(1,space);
//  back_right_wheel_control(0,space);
//}
//void  ce_move( int space_x , int space_y)
//{
//  if(space_x<0)
//	{
//	front_left_wheel_control(0,space_x*-1);
//	front_right_wheel_control(1,space_x*-1);
//	}
//	if(space_x>0)
//	{
//		front_left_wheel_control(1,space_x);
//		front_right_wheel_control(0,space_x);
//	} 
//	if(space_y<0)
//	{
//	back_left_wheel_control(0,space_y*-1);
//  back_right_wheel_control(0,space_y*-1);
//	}
//	if(space_y>0)
//	{
//	back_left_wheel_control(1,space_y);
//  back_right_wheel_control(1,space_y);
//	}
//}
