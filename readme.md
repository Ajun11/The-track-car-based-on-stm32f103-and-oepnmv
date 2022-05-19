# openmv追踪小车

## 一、openmv部分

### 1、舵机的跟随

云台追踪目标的原理是很简单的。openmv中搜索目标函数的返回值包括了目标物体中心的x、y坐标，原点是在图片的最左下角，就是说如果我们按照直接得到的坐标都是正的，但是我们要求云台追踪目标就是让目标始终出现在视野最中间，都是正的值我们无法判断图片到底是往哪边偏。为了解决这样的问题，我们只需要对得到的坐标进行简单的处理,openmv获得图片宽高都可以用函数获得，故已知图片宽width，高度height，目标中心点坐标x,y。按照相对比例来判断目标点在相机内的相对位置：
$$
y1=y/height-0.5
$$

$$
x1=x/width-0.5
$$

这样x1,y1就是我们最新获得的值，其取值范围均为[-0.5,0.5]。

为了实现云台始终追随目标，我们还需要将得到的坐标值转换为舵机旋转的角度，本实验云台为二自由度云台，如图1.1。下面的舵机控制偏航角与相机x轴相关，上面的舵机负责控制俯仰角与相机y轴相关，偏航角舵机的机械转角范围为[0,180],其中，当角度为0时，舵机朝向右侧，角度为180度时，舵机朝向左侧。俯仰角的机械转角范围为[90,180]，其中，当角度为90度时，平台成水平，当角度为180度时，平台垂直水平面。

<img src="C:\Users\LJ\Desktop\追踪小车\Laseted version programs\readme.imgs\1-16497681807631.jpg" style="zoom: 25%;" />

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

### 2、处理距离

选择识别的目标为AprilTag，在openmv中，有一个函数名为`find_AprilTag()`,具体使用可查看：[image — 机器视觉 — MicroPython 1.9.2 文档 (singtown.com)](https://docs.singtown.com/micropython/zh/latest/openmvcam/library/omv.image.html#image.Image.image.find_apriltags)此函数其返回对象的具体内容：可查看[image — 机器视觉 — MicroPython 1.9.2 文档 (singtown.com)](https://docs.singtown.com/micropython/zh/latest/openmvcam/library/omv.image.html#image.apriltag)。

我们只需要用到其返回的x,y,z坐标（相对于镜头中央），这三个需要的变量为索引12、13、14对应的值，其单位未知，但是与实际距离成一定的关系，这一比例可以用**K**来表示，对于不同大小的AprilTag其对应的K是不同的。
$$
diatance=K*(\sqrt(x^2+y^2+z^2))
$$
那么怎么测量属于我们自己的K呢，很简单。将openmv连接至电脑，在线调试，需要一把尺子，测量实际距离。将实际距离除以三者平方根值即可。目前，我们已经得到了一个距离物体的距离distance，对于追踪小车我们要发送给小车可以是直接的距离吗？当然不是，在第一步中我们设置openmv跟随物体，跟随时物体要处在镜头的正中央，舵机进行了角度的旋转，车呈现的状态，如图1.2

![](C:\Users\LJ\Desktop\追踪小车\Laseted version programs\readme.imgs\小车与物体.png)

​                                                                                     图1.2

所以说这个时候目标距离已知，我们舵机跟随的角度yaw_now,pitch_now已知，如何求出在实际空间中与物体之间的x、y距离？首先我们先不管x，y坐标距离究竟为何，设置默认坐标轴的方向，以车体为坐标轴，设置向右为x正方向，向前为y轴正方向，图1,2中已标出。

对小车距离进行分解，如图1.3给出了距离分解示意，原理还是很简单的，我们可看出首先应该将diatance分解到水平平面上，相当于空间在水平面上作出投影，得到水平距离distance_horizontal
$$
 distance_horizontal=distance*cos(pitch)
$$
![](C:\Users\LJ\Desktop\追踪小车\Laseted version programs\readme.imgs\小车分解.png)

通过水平距离我们就可以求得x,y方向坐标，坐标为
$$
（distance_horizontal*sin(yaw),distance_horizontal*cos(yaw))
$$
要注意的是，这里的yaw不是yaw_now,pitch也不是pitch_now，yaw_now,pitch_now均为舵机的旋转角，那么我们根据其几何特性：
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
这样我们openmv的任务就基本搞定，下面需要将消息通过串口发送出去。

### 3.串口发送消息

发送之前我们要考虑一下，我们都要向stm控制器传递什么？首先是距离目标的x,y坐标，能发送的前提是在视野内我们找到了目标，也就是我们还要发送一个目标是否找到的flag，找到时发送True，没有找到时发送False。
