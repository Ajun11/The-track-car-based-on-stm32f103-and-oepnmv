'''
1. 识别我的黄色水瓶或者APrilTag
2. 将识别的到的AprilTag位置,通过串口发送
'''
import sensor
import image
import time
import math
import ustruct as struct
from pyb import UART
from pyb import LED
from pyb import Servo

# 黄色水杯的LAB色彩空间阈值 (L Min, L Max, A Min, A Max, B Min, B Max)
YELLOW_PING_THRESHOLD = (6, 100, -128, 127, 31, 118)
# ROI搜索半径
ROI_R = 10

# 串口初始化
uart = UART(3, 115200)

#舵机的初始化
yaw_servo=Servo(1)
pitch_servo=Servo(2)

yaw_servo.calibration(500,2500,500)
pitch_servo.calibration(500,2500,500)

#pid控制相关变量以及函数
#偏航角角的pd.
Kp=17
Kd=3
last_bias=0#上次的误差这个需要保存所以要放在这里
yaw_now=90#没有舵机寻找程序就用90
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
    yaw_servo.angle(yaw_now,150)
#俯仰角pitch的控制pd
Kp_pitch=15
Kd_pitch=2
last_bias_pitch=0
pitch_now=135#没有舵机寻找程序就用135
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
yaw_servo.angle(yaw_now)
pitch_servo.angle(pitch_now,150)#初始角度，因为物理结构的原因这个150度正好朝前
# OpenMV感光芯片初始化
sensor.reset() # 重置感芯片
sensor.set_pixformat(sensor.RGB565) # 设置像素格式为RGB565
sensor.set_framesize(sensor.QQVGA) # 设置分辨率为QQVGA
sensor.set_vflip(True) # 纵向翻转
sensor.skip_frames(time = 2000) # 跳过2s内的帧, 等待画质稳定
sensor.set_auto_gain(False) # 关闭自动增益
sensor.set_auto_whitebal(False) # 关闭自动白平衡
#找最大块
def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob
# 初始化时钟
clock = time.clock()
#内循环break旗帜
break_flag=0
last_roi = None
#与目标点的距离
k=5.8#标准比例
distance=0#实际距离
get_x=0#目标点横向距离
get_y=0#目标点纵向距离
air_flag=0#判断舵机是否走到了极限
yaw_flag=0#偏航角是否走到了极限
find_flag=0
while(True):
    clock.tick() # 开始计时
    img = sensor.snapshot() # 拍摄一张照片

    blobs = []
    LED(3).on()
    blobs = img.find_apriltags()

    if len(blobs) == 0:
        # 获取画面中的色块
        #blobs = img.find_blobs([YELLOW_PING_THRESHOLD], pixels_threshold=100, area_threshold=100, merge=True)
        if(find_flag==1):
             uart.write(struct.pack('<BBBhh', 0xFF, 0xF1, False, 0, 0))
             time.sleep_ms(100)
        blobs = img.find_apriltags()
        #寻找后再次判断是否在目前视野，否则触发进入舵机寻找状态
        if len(blobs)==0:
            find_flag=0
            for i in range(36):
                if(yaw_now>=175):
                    yaw_flag=1
                elif(yaw_now<=0):
                    yaw_flag=0
                if(yaw_flag==1):
                    yaw_now=yaw_now-5
                else:
                    yaw_now=yaw_now+5
                yaw_servo.angle(yaw_now,120)
                time.sleep_ms(100)
                img = sensor.snapshot() # 拍摄一张照片
                blobs = img.find_apriltags()#拍摄后寻找一下
                if len(blobs)!=0:
                    print("色块丢失!!!")
                    last_roi = None
                    # 通过串口发送数据(二进制　低字节序)
                    uart.write(struct.pack('<BBBhh', 0xFF, 0xF1, False, 0, 0))
                    break
    if len(blobs) != 0:
        # 获得画面中的最大的色块
        blobs = img.find_apriltags()#拍摄后寻找一下
        blob =find_max(blobs)
        # 更新ROI
        last_roi = blob.rect()
        # 可视化
        img.draw_rectangle(blob.rect()) # 绘制色块的矩形区域
        img.draw_cross(int(blob[0]+blob[2]/2), int(blob[1]+blob[3]/2)) # 绘制色块的中心位置
        print("目标中心坐标: {} {}".format(blob[0]+blob[2]/2, blob[0]+blob[2]/2))
        yaw_error=(float(blob[0]+blob[2]/2)/float(img.width()))-0.5#此项为偏航误差
        pid_control(yaw_error)
        pitch_error=(float(blob[1]+blob[3]/2)/float(img.height()))-0.5#此项为俯仰误差
        pid_control_pitch(pitch_error)
        distance = math.sqrt(blob[12] ** 2 + blob[13] ** 2 + blob[14] ** 2)  # x,y,z坐标的平方和开根号求得距离
        distance = k*distance#与比例系数相乘得到实际的距离
        get_x=-1*math.cos(math.pi*pitch_now/180)*math.cos(math.pi*yaw_now/180)*distance#计算物体在空间中x,y距离
        get_y=-1*math.cos(math.pi*pitch_now/180)*math.sin(math.pi*yaw_now/180)*distance
        print("目标x坐标",get_x,"目标y坐标",get_y,"当前yaw",yaw_now,"pitch_now",pitch_now,"距离",distance)
        find_flag=1
        # 通过串口发送数据(二进制　低字节序)
        uart.write(struct.pack('<BBBhh', 0xFF, 0xF1, True, int(get_x), int(get_y)))
        print("send ok")
        #帧头1>帧头2>True>数据1的低八位>数据1的高八位>数据2的低八位>数据2的高八位
        #因为在openmv中存储的格式是小端，即低八位在低地址，高八位在高地址
    else:
        # 目标丢失
        print("色块丢失!!!")
        last_roi = None
        # 通过串口发送数据(二进制　低字节序)
        uart.write(struct.pack('<BBBhh', 0xFF, 0xF1, False, 0, 0))
    # 打印当前的帧率
    print(clock.fps())

    # pack各字母对应类型
    # x   pad byte        no value            1
    # c   char            string of length 1  1
    # b   signed char     integer             1
    # B   unsigned char   integer             1
    # ?   _Bool           bool                1
    # h   short           integer             2
    # H   unsigned short  integer             2
    # i   int             integer             4
    # I   unsigned int    integer or long     4
    # l   long            integer             4
    # L   unsigned long   long                4
    # q   long long       long                8
    # Q   unsilong long   long                8
    # f   float           float               4
    # d   double          float               8
    # s   char[]          string              1
    # p   char[]          string              1
    # P   void *          long
