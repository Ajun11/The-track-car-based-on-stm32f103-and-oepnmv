# 基于Openmv的视觉跟踪小车

## 一、实验原理及实验内容

### 1.物体识别

本次实验目的是使得小车可以跟踪目标，故首先确定跟踪目标，由于小车整体框架从零开始搭建，并没有太多的金钱可以选择昂贵的摄像头，故本次实验的目标识别选择较为简单的方式以减少硬件压力。本次实验首先识别纯色物体，是完成对纯色物体识别之后更进一步选择跟踪AprilTag。

AprilTag是一个视觉基准系统，可用于各种任务，包括AR，机器人和相机校准。这个tag可以直接用打印机打印出来，而AprilTag检测程序可以计算相对于相机的精确3D位置，方向和id。

AprilTag内容主要包含三个步骤：

第一步是如何根据梯度检测出图像中的各种边缘。

第二步即如何在边缘图像中找出需要的四边形图案并进行筛选，AprilTag尽可能的对检测出的边缘检测，首先剔除非直线边缘，在直线边缘进行邻接边缘查找，最终若形成闭环则为检测到一个四边形。对四边形进行解码确定Apriltag标签。

第三步确定四边形的中心点作为要跟踪的三维左边点。

Openmv对以上步骤进行了函数封装，可以用img.find_apriltags()函数定位Apriltag标签，并且可以通过该函数的返回值的方法确定三维坐标和三维角度：可以用获取x轴坐标tag.x_translation(), tag.y_translation()、tag.z_translation()是y、z轴坐标 。

### 2.云台追踪

openmv中搜索目标函数的返回值包括了目标物体中心的x、y坐标，原点是在图片的最左下角，就是说如果我们按照直接得到的坐标都是正的，但是我们要求云台追踪目标就是让目标始终出现在视野最中间，都是正的值我们无法判断图片到底是往哪边偏。为了解决这样的问题，我们只需要对得到的坐标进行简单的处理,openmv获得图片宽高都可以用函数获得，故已知图片宽width，高度height，目标中心点坐标x,y。按照相对比例来判断目标点在相机内的相对位置：
$$
y1=y/height-0.5
$$

$$
x1=x/width-0.5
$$

这样x1,y1就是我们最新获得的值，其取值范围均为[-0.5,0.5]。

为了实现云台始终追随目标，我们还需要将得到的坐标值转换为舵机旋转的角度，本实验云台为二自由度云台，如图1.1。下面的舵机控制偏航角与相机x轴相关，上面的舵机负责控制俯仰角与相机y轴相关，偏航角舵机的机械转角范围为[0,180],其中，当角度为0时，舵机朝向右侧，角度为180度时，舵机朝向左侧。俯仰角的机械转角范围为[90,180]，其中，当角度为90度时，平台成水平，当角度为180度时，平台垂直水平面。

<img src="readme.imgs/1-16497681807631.jpg" style="zoom: 25%;" />

​                                                                                  **图1.1**

算法上的实现，算法上的实现可以使用pd控制，pd控制较为稳定更适合舵机。我们已知（x1,y1）为当前目标的坐标，目的是将其移动到镜头中央，那么目标点为（0,0）,我们获得了x轴的偏差以及y轴的偏差error_x,error_y上一次x,y偏差为error_x_last,error_y_last。假设此时舵机角度为yaw_now,pitch_now,那么有简化版增量式pd算法为：
$$
output_x=Kp*（error_x ）+Kd*(error_x-error_(x_last ))
$$

$$
output_y=Kp_y*(error_y )+Kd*(error_y-error_(y_last ))
$$

则当前获得偏航角
$$
yaw_now+=output_x
$$
获得当前俯仰角
$$
pitch_now+=output_y
$$
让两个舵机执行对应的角度，实现控制，过程中需要不断调节Kp，Kd参数，Kp调节参数方式：逐渐增大Kp，使舵机出现抖动，再逐渐降低Kp值，使系统逐渐趋于稳定，Kd的调参方式与此相似。

### 3.与目标直线距离的几何化处理

选择识别的目标为AprilTag，在openmv中，有一个函数名为`find_AprilTag()`,具体使用可查看：[image — 机器视觉 — MicroPython 1.9.2 文档 (singtown.com)](https://docs.singtown.com/micropython/zh/latest/openmvcam/library/omv.image.html#image.Image.image.find_apriltags)此函数其返回对象的具体内容：可查看[image — 机器视觉 — MicroPython 1.9.2 文档 (singtown.com)](https://docs.singtown.com/micropython/zh/latest/openmvcam/library/omv.image.html#image.apriltag)。

我们只需要用到其返回的x,y,z坐标（相对于镜头中央），这三个需要的变量为索引12、13、14对应的值，其单位未知，但是与实际距离成一定的关系，这一比例可以用**K**来表示，对于不同大小的AprilTag其对应的K是不同的。
$$
diatance=K*(\sqrt(x^2+y^2+z^2))
$$
那么怎么测量属于我们自己的K呢，很简单。

#### K值的计算

OpenMV采用的是单目摄像头，想要实现测距，就需要选参照物，利用参照物的大小比例来计算距离。

![clip_image002](../readme.imgs/clip_image002.png)

Lm为物体距摄像头的距离，L’为焦距，Bx为物体在图像中的直径像素，Ax为图像的直径像素，Rm为物体实际大小的半径，Hm为摄像头可拍摄的半径。

![clip_image002](../readme.imgs/clip_image002.jpg)

求常数k的方法是：先让物体距离摄像头10cm，打印出摄像头里直径的像素值，然后将像素值与距离相乘，就得到了k的值。

由于打印的AprilTag 标签大小不定，所以img.find_apriltags()函数返回参与单位也不定，确定单位方法与确定常数K的方法一致。

目前，我们已经得到了一个距离物体的距离distance，对于追踪小车我们要发送给小车可以是直接的距离吗？当然不是，在第一步中我们设置openmv跟随物体，跟随时物体要处在镜头的正中央，舵机进行了角度的旋转，车呈现的状态，如图1.2

![](readme.imgs/%E5%B0%8F%E8%BD%A6%E4%B8%8E%E7%89%A9%E4%BD%93.png)

​                                                                                     图1.2

所以说这个时候目标距离已知，我们舵机跟随的角度yaw_now,pitch_now已知，如何求出在实际空间中与物体之间的x、y距离？首先我们先不管x，y坐标距离究竟为何，设置默认坐标轴的方向，以车体为坐标轴，设置向右为x正方向，向前为y轴正方向，图1.2中已标出。

对小车距离进行分解，如图1.3给出了距离分解示意，原理还是很简单的，我们可看出首先应该将diatance分解到水平平面上，相当于空间在水平面上作出投影，得到水平距离distance_horizontal
$$
distance_horizontal=distance*cos(pitch)
$$
![](readme.imgs/%E5%B0%8F%E8%BD%A6%E5%88%86%E8%A7%A3.png)

通过水平距离我们就可以求得x,y方向坐标，坐标为
$$
（distance_horizontal*sin(yaw),distance_horizontal*cos(yaw))
$$
要注意的是，这里的yaw不是yaw_now,pitch也不是pitch_now，yaw_now,pitch_now均为舵机目前的旋转角，那么我们根据其机械几何特性（搞不懂请看：2.云台追踪）：
$$
pitch=180-pitch_now
$$

$$
yaw=yaw_now-90
$$

更新x,y坐标表示：
$$
(x,y)=（distance*cos(pitch)*sin(yaw),  distance*cos(pitch)*cos(yaw))=(-distance*cos(pitch_now)*cos(yaw_now),-distance*cos(pitch_now)*sin(yaw_now))
$$
这样我们openmv的任务就基本搞定，得到的x,y即为物体空间坐标表示，下面需要将消息通过串口发送出去。

### 4.串口消息的发送与接收

#### （1）连线：

##### 对于stm32

选择stm32的USART2与openmv的USART3相连接，其中stm32的USART2:

![image-20220601092258959](readme.imgs/image-20220601092258959-16540513550271.png)

在上图中可以看出PA2为信号发送端口，PA3为接收端口。

##### 对于openmv：

图中的openmv为ov7725，我们购买的摄像头为ov5640,其中usart3对应的是p4、p5引脚，p4引脚对应的是发送端口，p5端口对应的是接收端口。

##### 二者的连接：

stm32的发送端口连接openmv的接收端口，stm的接收端口连接openmv的发送端口。也就是说：stm32的PA2连接openmv的P5引脚，stm32的PA3连接openmv的P4引脚。

#### （2）软件编写：

##### Openmv端的发送

启用uart3作为串口发送，并设置波特率115200

`uart = UART(3, 115200)`

uart.write是openmv的UART库中写好的函数，可以直接调用。这里面的#struct.pack是将信息打包,pack各字母对应类型:

| 字母       | 变量           | 类型               | 字节长度 |
| ---------- | -------------- | ------------------ | -------- |
| x          | pad byte       | no value           | 1        |
| c          | char           | string of length 1 | 1        |
| b          | signed char    | integer            | 1        |
| B          | unsigned char  | integer            | 1        |
| ?          | _Bool          | bool               | 1        |
| h          | short          | integer            | 2        |
| H          | unsigned short | integer            | 2        |
| i          | int            | integer            | 4        |
| I          | unsigned int   | integer or long    | 4        |
| L （小写l) | long           | integer            | 4        |
| L          | unsigned long  | long               | 4        |
| q          | long long      | long               | 8        |
| Q          | unsilong long  | long               | 8        |
| f          | float          | float              | 4        |
| d          | double         | float              | 8        |
| s          | char[]         | string             | 1        |
| p          | char[]         | string             | 1        |
| P          | void *         | long               |          |

`uart.write(struct.pack('<BBBhh', 0xFF, 0xF1, Bool_s, int(get_x), int(get_y)))`

’BBBhh’代表：字符+字符+字符+短整型+短整型

- '0xFF, 0xF1'两个帧头   
- 'Bools':是否找到目标     
- 'get_x   gety':发送目标x、y坐标        

##### Stm32端接收

Stm32端在进行接收时将会进入中断函数，stm32的中断函数名是固定的，必须要按照其规定要求进行使用，本次使用的是USART2串口，所以使用的中断函数为：

`void USART2_IRQHandler(void)；`

每一次中断函数接收到为一字节数据，也就是说，得到的数据需要我们进行存储，等到一帧数据获取完毕后，判断数据的帧头，在这次实验中，帧头为0XFF，0XF1，获取到帧头后才可以对剩余的数据进行处理，将得到的数据存储为静态变量，以方便外部函数调用对数值进行处理。

### 5.PID算法控制

得到的x、y坐标均是以小车的摄像头为原点，所以想要达到目标点，当前对于小车的x、y的误差即为x、y的坐标。x轴的输出，其输出值对应的是pwm波增量（相对于可以使电机产生运动的基准增量）

`output_x=Kp_x*(error_x_now)+Kd_x*(error_x_last-error_x_now);`

y轴的输出同理。

对于与目标距离，应允许其误差存在，不然会撞上，在这段误差范围内将输出调整至0，以满足停止的要求。假设output_x最终被判为0，但是output_y不为0，那么小车此时的运动应为直线前进（横向为x，向前为y）。

假设ouput_x不为0，那么无论如何，应让车进行旋转调整（边走边转因为速度原因很容易丢失目标），通过差速的方式对车进行控制，使得output_x为0，这样小车就可以继续前进。

## 二、实验心得与收获

这次实验是第一次小组成员深入接触硬件，没有选择过于强悍的开发板树莓派、Jeston nano，也没有选择最近叱咤风云的linux、ros。选择了最基础的stm32f103底控和看似不那么聪明的Openmv（主控ST)，电机驱动选择了L298N（ST的），说是一辆基于*ST*公司的智能车也不为过。接触底层，电机的转动有时也会出现点小插曲，经过这一次实践之后，对于stm32的认识更加深刻，对于其时钟等操作函数的掌握也更加牢固。第一次接触Openmv，不得不说的是，摄像头的帧率是真的高，处理速度真的是快，这也是能让整体控制系统更加丝滑的原因之一。总而言之，这次实验带来的收获无比巨大。

## 三、部分代码以及效果展示

（这这这大家看不看都行奥这个！！！！）

### 1.测距的函数

我们这里假设物体为红色，可以用识别颜色的函数作为检测物体

```python
K=10
blobs = img.find_blobs([red])
    if len(blobs) == 1:
        b = blobs[0]
        Lm = (b[2]+b[3])/2
        length = K/Lm
        print(length)
#Lm用物体在摄像头的投影的长和宽的平均值作为直接像素大小。
```

### 2.识别AprilTag并测距

```python
for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y):         img.draw_rectangle(tag.rect(), color = (255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
        k=2.4#比例尺           s=math.sqrt(pow(tag.x_translation(),2)+pow(tag.y_translation(),2)+pow(tag.z_translation(),2))
        length=s*k#利用成像原理数学公式计算真实长度
        print("length= ",length)
```

### 3.云台跟踪代码

```python
#yaw角的pid 
Kp=14 
Kd=2 
last_bias=0#上一时刻的误差需要保存
yaw_now=0#没有舵机寻找程序就用90 
yaw_error=0 
def pid_control(bias): 
    global KP,Kd,last_bias,yaw_now 
    output=Kp*bias+Kd*(last_bias-bias) 
    last_bias=bias 
    yaw_now=yaw_now+output 
    if yaw_now>179: 
        yaw_now=179 
    elif yaw_now<0: 
        yaw_now=0 
    yaw_servo.angle(yaw_now) 
#俯仰角pitch的控制pid 
Kp_pitch=13.5 
Kd_pitch=2 
last_bias_pitch=0 
pitch_now=90#没有舵机寻找程序就用135 
pitch_error=0 
def pid_control_pitch(bias): 
    global Kp_pitch,Kd_pitch,last_bias_pitch,pitch_now 
    output=Kp_pitch*bias+Kd*(last_bias_pitch-bias) 
    last_bias_pitch=bias 
    pitch_now=pitch_now+output 
    if pitch_now>179: 
        pitch_now=179 
    elif pitch_now<90: 
        pitch_now=90 
    print("pitch_now",pitch_now) 
pitch_servo.angle(pitch_now)
```

### 4.Openmv发送串口消息

```python
# 串口初始化
uart = UART(3, 115200)
# 通过串口发送数据(二进制　低字节序)
uart.write(struct.pack('<BBBhh', 0xFF, 0xF1, True, int(get_x), int(get_y)))
print("send ok")
```

### 5.Stm32串口处理函数

```C
//接收处理函数
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
uint16_t uart2Available(void)//
{
    return h_u32RecCnt;
}
void USART2_IRQHandler(void)//中断函数
{ //千万不能用USART_GetITStatus
	if(USART_GetFlagStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断有效,若接收数据寄存器满
	{
		rx2Buffer[rxBufferNum] = USART_ReceiveData(USART2); //接收8 Bit数据 ,例如0x03
		rxBufferNum++;
		if (rxBufferNum >= UART2_BUFFER_SIZE ) rxBufferNum = 0;//这里的判断仅仅是设置存储的限制而已，即此数组可以一共存储32组数据
		h_u32RecCnt++;
 } 
```

### 6.Stm pid控制函数

```C
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
          front_left_wheel_control(1,0);
          front_right_wheel_control(1,0);
          back_left_wheel_control(1,0);
          back_right_wheel_control(1,0);
		}
		else if(output_y!=0)
        {

          front_left_wheel_control(1,standard_pwm+output_y);
          front_right_wheel_control(1,standard_pwm+output_y);
          back_left_wheel_control(1,standard_pwm+output_y);
          back_right_wheel_control(1,standard_pwm+output_y);
        }
	}
	else if(output_x!=0)
	{
	  front_left_wheel_control(1,standard_pwm+output_x);
	  front_right_wheel_control(1,standard_pwm-output_x);
	  back_left_wheel_control(1,standard_pwm+output_x);
	  back_right_wheel_control(1,standard_pwm-output_x);
	}
#endif
	}
}
```

