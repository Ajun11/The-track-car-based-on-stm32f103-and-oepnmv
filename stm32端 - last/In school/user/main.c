#include 					"stm32f10x.h"
#include 					"bsp_led.h"
#include 					"bsp_motor.h"
#include          "stdio.h"
#include          "usart.h"
#include          "stdlib.h"
#define           OPEN     1
extern short      x_axis,y_axis;
extern int        blob_flag;
unsigned int      standard_pwm=600;
unsigned int      standard_output=60;
int               Kp_x=5;
int               Kp_y=5;
int               Kd_x=2;
int               Kd_y=2;
int               output_x=0,output_y=0;
int               error_x_now=0,error_x_last=0,error_y_now=0,error_y_last=0;
int               P_value ,v=1;
/****************剩下的就是调参了***************/
void car_init()
{
	LED_GPIO_Config();
	MOTOR_INIT();
	USART2_Init(115200);
}
int main(void)
{
	car_init();
	LED1_OFF;
	while(1)
	{
	 TR_Receive(); 
#if defined OPEN	
	error_x_now=x_axis;
	error_y_now=y_axis;
	output_x=Kp_x*(error_x_now)+Kd_x*(error_x_last-error_x_now);
	error_x_last=error_x_now;
	if(output_x>90)output_x=90;
	if(output_x<-90)output_x=-90;
	if(abs(error_x_now)<10)output_x=0;
	
	output_y=Kp_y*(error_y_now)+Kd_y*(error_y_last-error_y_now);
	error_y_last=error_y_now;
	if(output_y>90)output_y=90;//pwm输出限幅
	if(output_y<0)output_y=0;
	if(abs(error_y_now)<10)output_y=0;//误差允许范围
	
	if(blob_flag==0)//是否检测到物块，没有先停车
	{
		output_x=0;
		output_y=0;
	}
	//	//以下
	if(output_x==0)
	{
		if(output_y==0)
		{
 		P_value=1;
	  front_left_wheel_control(1,0);
 	  front_right_wheel_control(1,0);
  	back_left_wheel_control(1,0);
	  back_right_wheel_control(1,0);
		}
		else if(output_y!=0)
	{
		P_value=2;
		front_left_wheel_control(1,standard_pwm+output_y);
 	  front_right_wheel_control(1,standard_pwm+output_y);
  	back_left_wheel_control(1,standard_pwm+output_y);
	  back_right_wheel_control(1,standard_pwm+output_y);
	}
	}
	else if(output_x!=0)
	{
		v=20;
		if(output_y!=0)
		{
			P_value=3;
			front_left_wheel_control(1,standard_pwm+output_x);
			front_right_wheel_control(1,standard_pwm-output_x);
			back_left_wheel_control(1,standard_pwm+output_x);
			back_right_wheel_control(1,standard_pwm-output_x);
		}
		if(output_y==0)
		{
			P_value=4;
			front_left_wheel_control(1,standard_pwm+output_x);
			front_right_wheel_control(1,standard_pwm-output_x);
			back_left_wheel_control(1,standard_pwm+output_x);
			back_right_wheel_control(1,standard_pwm-output_x);
			
		}
	
	}
#endif
	}
}
