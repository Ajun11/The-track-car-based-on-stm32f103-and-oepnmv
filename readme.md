# openmv追踪小车

## openmv部分

云台追踪目标的原理是很简单的。openmv中搜索目标函数的返回值包括了目标物体中心的x、y坐标，原点是在图片的最左下角，就是说如果我们按照直接得到的坐标都是正的，但是我们要求云台追踪目标就是让目标始终出现在视野最中间，都是正的值我们无法判断图片到底是往哪边偏。为了解决这样的问题，我们只需要对得到的坐标进行简单的处理,openmv获得图片宽高都可以用函数获得，故已知图片宽width，高度height，目标中心点坐标x,y。按照相对比例来判断目标点在相机内的相对位置：
$$
y1=y/height-0.5
$$

$$
x1=x/width-0.5
$$

这样x1,y1就是我们最新获得的值，其取值范围均为[-0.5,0.5]。

为了实现云台始终追随目标，我们还需要将得到的坐标值转换为舵机旋转的角度，本实验云台为二自由度云台，如图2.1。下面的舵机控制偏航角与相机x轴相关，上面的舵机负责控制俯仰角与相机y轴相关，偏航角舵机的机械转角范围为[0,180],其中，当角度为0时，舵机朝向左侧，角度为180度时，舵机朝向右侧。俯仰角的机械转角范围为[90,180]，其中，当角度为90度时，平台成水平，当角度为180度时，平台垂直水平面。

![](C:\Users\LJ\Desktop\董晨皓\Laseted version\readme.imgs\1-16497681807631.jpg)

**图2.1**

算法上的实现，算法上的实现可以使用pd控制，pd控制较为稳定更适合舵机。我们已知（x1,y1）为当前目标的坐标，目的是将其移动到镜头中央，那么目标点为（0,0）,我们获得了x轴的偏差以及y轴的偏差error_x,error_y上一次x,y偏差为error_x_last,error_y_last。假设此时舵机角度为yaw_now,pitch_now,那么有增量式pd算法为：
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

Openmv中控制部分代码：

```python
#yaw角的pid
Kp=14
Kd=2
last_bias=0#上次的误差这个需要保存所以要放在这里，我真是服了python哎
yaw_now=0#没有舵机寻找程序就用90
yaw_error=0
def pid_control(bias):
    global KP,Kd,last_bias,yaw_now,yaw_error
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
    global Kp_pitch,Kd_pitch,last_bias_pitch,pitch_now,pitch_error
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

