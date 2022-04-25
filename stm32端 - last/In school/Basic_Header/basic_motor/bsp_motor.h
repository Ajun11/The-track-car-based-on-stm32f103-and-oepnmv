#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H
#include "stm32f10x.h"
/************高级定时器TIM参数定义，小小核心板只限TIM1************/
//可参照头文件199行
// 当使用不同的定时器的时候，对应的GPIO是不一样的，高级定时器是在高速总线APB1上，但是别的不是，这点要注意
// 这里我们使用高级控制定时器TIM1
#define            ADVANCE_TIM_APBxClock_FUN_1     RCC_APB2PeriphClockCmd
#define            ADVANCE_TIM_1                   TIM1
#define            ADVANCE_TIM_CLK_1               RCC_APB2Periph_TIM1
// PWM 信号的频率 F = TIM_CLK/{(ARR+1)*(PSC+1)}
#define            TIM_PERIOD                      799
#define            TIM_PSC                         8
/**************************函数声明********************************/

void GENERAL_TIM_Init(void);
/*轮子端口定义*/
//这里由于我以前对L298N理解有误，发现其实在两端的跳帽口输入pwm依然可以起到控制的作用，只是说那四个引脚要有高低的区别，他们之间高低差别带动电机运动方向。
//在此对此头文件进行修改
//下面四个引脚是属于时钟1的四个通道输出端口，输出四路pwm
#define           left_front_wheel_port     		 GPIOA
#define           left_front_wheel_pin        	 GPIO_Pin_8
#define           left_front_wheel_clk           RCC_APB2Periph_GPIOA
  
#define           right_front_wheel_port         GPIOA
#define           right_front_wheel_pin       	 GPIO_Pin_9
#define           right_front_wheel_clk          RCC_APB2Periph_GPIOA

#define           left_back_wheel_port     		   GPIOA
#define           left_back_wheel_pin        	   GPIO_Pin_10
#define           left_back_wheel_clk            RCC_APB2Periph_GPIOA
  
#define           right_back_wheel_port          GPIOA
#define           right_back_wheel_pin           GPIO_Pin_11
#define           right_back_wheel_clk           RCC_APB2Periph_GPIOA
//下面八个是控制引脚高低电平的普通引脚。
//左前轮的两个控制口******************////
///Reset输出低电平，Set输出高电平////////
///L298N从左到右In 1 2 3 4 1若输出高则正转///
#define           left_front_wheel_control_1_port    GPIOB  //298N  in1
#define           left_front_wheel_control_1_pin     GPIO_Pin_11
#define           left_front_wheel_control_1_clk     RCC_APB2Periph_GPIOB
#define           left_front_wheel_control_1_low     GPIO_ResetBits(GPIOB,GPIO_Pin_11)
#define           left_front_wheel_control_1_high    GPIO_SetBits(GPIOB,GPIO_Pin_11)

#define           left_front_wheel_control_2_port    GPIOB  //2
#define           left_front_wheel_control_2_pin     GPIO_Pin_10
#define           left_front_wheel_control_2_clk     RCC_APB2Periph_GPIOB
#define           left_front_wheel_control_2_low     GPIO_ResetBits(GPIOB,GPIO_Pin_10)
#define           left_front_wheel_control_2_high    GPIO_SetBits(GPIOB,GPIO_Pin_10)

//右前轮的两个控制口******************///////
#define           right_front_wheel_control_1_port   GPIOB  //3
#define           right_front_wheel_control_1_pin    GPIO_Pin_1
#define           right_front_wheel_control_1_clk    RCC_APB2Periph_GPIOB
#define           right_front_wheel_control_1_low    GPIO_ResetBits(GPIOB,GPIO_Pin_1)
#define           right_front_wheel_control_1_high   GPIO_SetBits(GPIOB,GPIO_Pin_1)

#define           right_front_wheel_control_2_port   GPIOB //4
#define           right_front_wheel_control_2_pin    GPIO_Pin_0
#define           right_front_wheel_control_2_clk    RCC_APB2Periph_GPIOB
#define           right_front_wheel_control_2_low    GPIO_ResetBits(GPIOB,GPIO_Pin_0)
#define           right_front_wheel_control_2_high   GPIO_SetBits(GPIOB,GPIO_Pin_0)
//左后轮的两个控制口
#define           left_back_wheel_control_1_port     GPIOB
#define           left_back_wheel_control_1_pin      GPIO_Pin_6
#define           left_back_wheel_control_1_clk      RCC_APB2Periph_GPIOB
#define           left_back_wheel_control_1_low      GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define           left_back_wheel_control_1_high     GPIO_SetBits(GPIOB,GPIO_Pin_6)

#define           left_back_wheel_control_2_port     GPIOB
#define           left_back_wheel_control_2_pin      GPIO_Pin_7
#define           left_back_wheel_control_2_clk      RCC_APB2Periph_GPIOB
#define           left_back_wheel_control_2_low      GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define           left_back_wheel_control_2_high     GPIO_SetBits(GPIOB,GPIO_Pin_7)

//右后轮的两个控制口
#define           right_back_wheel_control_1_port    GPIOB
#define           right_back_wheel_control_1_pin     GPIO_Pin_8
#define           right_back_wheel_control_1_clk     RCC_APB2Periph_GPIOB
#define           right_back_wheel_control_1_low     GPIO_ResetBits(GPIOB,GPIO_Pin_8)
#define           right_back_wheel_control_1_high    GPIO_SetBits(GPIOB,GPIO_Pin_8)

#define           right_back_wheel_control_2_port    GPIOB
#define           right_back_wheel_control_2_pin     GPIO_Pin_9
#define           right_back_wheel_control_2_clk     RCC_APB2Periph_GPIOB
#define           right_back_wheel_control_2_low     GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define           right_back_wheel_control_2_high    GPIO_SetBits(GPIOB,GPIO_Pin_9)
/**************************函数声明********************************/

//judge_value判断前后，Space为CRR值 要求：CRR<800
//左前轮前转后转以及速度
void  front_left_wheel_control(int judge_value,unsigned int Space);
void  front_right_wheel_control(int judge_value,unsigned int Space);
void  back_left_wheel_control(int judge_value,unsigned int Space);
void  back_right_wheel_control(int judge_value,unsigned int Space);
void  MOTOR_INIT(void);
#endif
