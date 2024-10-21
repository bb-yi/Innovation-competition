from pyb import LED
import sensor
import image
import time
import pyb
import math
from machine import UART
uart = UART(3, baudrate=115200)
#red_led = LED(1)
#green_led = LED(2)
#blue_led = LED(3)


# 颜色识别使用阈值：
thresholds = [
    (43, 56, 5, 18, 2, 6),        # 红色R
    (54, 68, -7, 14, -24, -6),  # 蓝色G
    (43, 58, -26, -13, 5, 12)  # 绿色B(42, 60, -27, -10, 1, 15)
]


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
# sensor.set_vflip(True)
# sensor.set_hmirror(True)
# QQQVGA=80*60  分辨率 寻线
# QQVGA=160x120 分辨率 颜色
# QVGA =320*240 分辨率 形状
sensor.skip_frames(20)
clock = time.clock()                 # 初始化时钟对象


# aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
# 红色1 绿色2 蓝色3
# 发送二维码字符串数据
# 获得全局变量整数类型
qrcode = []


def get_qrcode():
    global qrcode
    sensor.set_auto_gain(False)
    img = sensor.snapshot().lens_corr(1.8)
    for code in img.find_qrcodes():
        img.draw_rectangle(code.rect(), color=127)
        msg = code.payload()
        if msg != None:
            qrcode = [msg]
            print('qrcode:', qrcode)  # 打印二维码数据
            msg = msg.replace('+', ',')
            msg2 = f"[e,{msg}]"
            uart.write(msg2+'\r\n')  # 发送的是字符串类型
            img.draw_string(50, 50, msg2, color=127)


# bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb


def get_color():
    img = sensor.snapshot()
    x1 = 40
    y1 = 40
    w = 80
    h = 50
    rect_tuple = (x1, y1, w, h)
    img.draw_rectangle(rect_tuple, color=(0, 255, 255))
    for i, threshold in enumerate(thresholds):
        color_name = ["1", "3", "2"][i]
        blobs = img.find_blobs([threshold], roi=rect_tuple)
        if blobs:
            max_blob = find_max1(blobs)
            if max_blob is not None:
                img.draw_rectangle(max_blob.rect(), color=(255, 0, 0))
                img.draw_cross((max_blob.cx(), max_blob.cy()), size=5, color=(255, 0, 0))
                print(f'{color_name} 找到')
                t = f'[c,{color_name}]'
                uart.write(t+'\r\n')


def find_max1(blobs):
    max_size = 0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob = blob
            max_size = blob[2]*blob[3]
    return max_blob


def determine_color(color, threshold=50):
    r, g, b = color  # 获取颜色的 R、G、B 值

    # 容错处理，通过阈值比较通道值
    if r > g + threshold and r > b + threshold:
        return 1  # 红色
    elif g > r + threshold and g > b + threshold:
        return 2  # 绿色
    elif b > r + threshold and b > g + threshold:
        return 3  # 蓝色
    else:
        return 0  # 如果没有明显的主颜色，则判断为不确定
def get_dominant_color_in_roi(img,roi, current_x, current_y, Radius):

    '''场地颜色阈值
    red_threshold = (24, 58, 11, 81, -52, 77)# 红色阈值
    green_threshold = (42, 60, -34, -4, -1, 108)  # 绿色阈值
    blue_threshold = (18, 50, -5, 13, -39, -8)      # 蓝色阈值
    '''
    red_threshold = (58, 70, -2, 31, -16, 21)# 红色阈值
    green_threshold = (46, 68, -19, 5, -13, 18)  # 绿色阈值
    blue_threshold = (56, 69, -4, 13, -23, -10)      # 蓝色阈值

    # 查找红、绿、蓝色块
    red_blobs = img.find_blobs([red_threshold], roi=roi, pixels_threshold=10, area_threshold=10, merge=True)
    green_blobs = img.find_blobs([green_threshold], roi=roi, pixels_threshold=10, area_threshold=10, merge=True)
    blue_blobs = img.find_blobs([blue_threshold], roi=roi, pixels_threshold=10, area_threshold=10, merge=True)

    # 初始化加权面积
    red_weighted_area, green_weighted_area, blue_weighted_area = 0, 0, 0

    # 定义新的权重函数：距离越远，权重越小，遵循 x^0.5 规律
    def calculate_weight(blob_x, blob_y):
        distance = math.sqrt((blob_x - current_x) ** 2 + (blob_y - current_y) ** 2)
        if distance > Radius:
            return 0  # 超出半径范围的权重为 0
        # 使用 sqrt(distance) 规律，距离越近，权重越小
        return math.sqrt(distance) / math.sqrt(Radius)

    # 计算红色块加权面积
    for blob in red_blobs:
        weight = calculate_weight(blob.cx(), blob.cy())
        red_weighted_area += blob.area() * weight

    # 计算绿色块加权面积
    for blob in green_blobs:
        weight = calculate_weight(blob.cx(), blob.cy())
        green_weighted_area += blob.area() * weight

    # 计算蓝色块加权面积
    for blob in blue_blobs:
        weight = calculate_weight(blob.cx(), blob.cy())
        blue_weighted_area += blob.area() * weight
    print({red_weighted_area},{green_weighted_area},{blue_weighted_area})
    # 判断加权面积最大的颜色
    if red_weighted_area > green_weighted_area and red_weighted_area > blue_weighted_area:
        return (255, 0, 0)  # 红色
    elif green_weighted_area > red_weighted_area and green_weighted_area > blue_weighted_area:
        return (0, 255, 0)  # 绿色
    elif blue_weighted_area > red_weighted_area and blue_weighted_area > green_weighted_area:
        return (0, 0, 255)  # 蓝色
    else:
        return (100, 100, 100)  # 无明显主颜色


def calculate_roi(center_x, center_y, width, height):
    # 计算矩形的左上角坐标
    x = int(center_x - width / 2)
    y = int(center_y - height / 2)

    # 返回 ROI，格式为 (x, y, w, h)
    return (x, y, width, height)
def find_max(circles):
    """ 从给定的圆列表中找到半径最大的圆。 """
    if not circles:
        return None

    max_circle = max(circles, key=lambda c: c.r())
    return max_circle

# 初始化全局变量
last_x, last_y = 240, 160
max_deviation = 40  # 允许的最大偏差值（可以根据需求调整）
last_Radius = 10
# roi = calculate_roi(80, 60, 160, 120)  # QQVGA
# roi = calculate_roi(160, 120, 320, 240)#QVGA
# roi = calculate_roi(320, 240, 640, 480)#VGA
roi = calculate_roi(240, 160, 360, 240)  # HVGA
color = (0,0,0)

def get_circles(max_x=160, max_y=120, threshold=2000, r_min=8, r_max=16):
    global last_x, last_y  # 声明为全局变量
    global last_Radius  # 声明为全局变量
    global roi  # 声明为全局变量
    global color
    img = sensor.snapshot()
    width = img.width()
    height = img.height()
    circles = img.find_circles(roi=roi, threshold=threshold, x_margin=10, y_margin=10, r_margin=10, r_min=r_min, r_max=r_max, r_step=1)

    if circles:
        #c = circles[0]  # 获取第一个圆
        c = find_max(circles    )  # 获取第一个圆
        pos_alpha = 0.5
        current_x = int(c.x() * pos_alpha + last_x * (1 - pos_alpha))
        current_y= int(c.y() * pos_alpha + last_y * (1 - pos_alpha))
        Radius = int(c.r() * 0.2 + last_Radius * 0.8)
        # 计算与上一次圆心位置和半径的偏差
        deviation = math.sqrt((current_x - last_x) ** 2 + (current_y - last_y) ** 2)
        radius_deviation = abs(Radius - last_Radius)
        # 如果偏差过大，忽略本次更新
        if deviation > max_deviation:
            #print(f"圆心或半径变化过大，忽略本次结果。偏差: {deviation}, 半径偏差: {radius_deviation}")
            last_x = current_x
            last_y = current_y
            return  # 跳过当前结果
        if (max_x != 160):#更改范围降低性能消耗
            roi = calculate_roi(current_x, current_y, 120, 120)

        if (max_x == 160):
            color = img.get_pixel(current_x, current_y)
        else:
            color = get_dominant_color_in_roi(img,roi, current_x, current_y, Radius-5)
            #color =(0,0,0)
        #prev_time = current_time
        # 输出圆心位置及颜色信息
        message = f"[y,{current_x/width:.4f},{current_y/height:.4f},{determine_color(color,20)}]"  # 消息内容

        # 画出圆心和圆
        img.draw_circle(current_x, current_y, Radius, color=(255, 0, 0))
        img.draw_cross(current_x, current_y, size=5, color=(255, 0, 0))  # 传入两个独立坐标
        # 更新上一次的圆心位置和半径
        last_x, last_y = current_x, current_y
        last_Radius = Radius
    else:
        # roi = calculate_roi(max_x/2, max_y/2, max_x, max_y)
        if(max_x == 160):
            roi = calculate_roi(80, 60, 160, 120)
        else:
            roi = calculate_roi(max_x/2, max_y/2, 280, 210)  # HVGA

        message = "NONE"
    img.draw_rectangle(roi, color=(0, 255, 0), thickness=1)  # 绘制绿色矩形表示 ROI
    print(message)
    uart.write(message+'\r\n')  # 发送字节串





# dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd

# theta代表返回线段的角度, rho代表偏移的距离
# rho_err 左右偏移偏差值 即小车越接近为负数，远离为正数，正中间为0
# theta_err 角度偏差值 向左偏为负数，向右偏为正数，正中间为0

# 巡线的二值化所用阈值
THRESHOLD = (45, 82, -15, 11, 1, 51)# 白色纸面灰色线识别阈值
# 黄色边框条(46, 78, -4, 25, 4, 38)

# 定义滤波系数，可以根据实际情况调整
alpha_rho = 0.5
alpha_theta = 0.5

# 初始化上一时刻的滤波输出值
prev_rho_err_filtered = 0
prev_theta_err_filtered = 0

def seek_line():
    global prev_rho_err_filtered
    global prev_theta_err_filtered
    img = sensor.snapshot()
    #print(img.width())
    img.binary([THRESHOLD])
    line = img.get_regression([(100,100)], robust = True)
    if (line):
        rho_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color = 127)

        # 应用一阶低通滤波器
        rho_err_filtered = alpha_rho * (rho_err + 30) / 60 + (1 - alpha_rho) * prev_rho_err_filtered
        theta_err_filtered = alpha_theta * theta_err + (1 - alpha_theta) * prev_theta_err_filtered

        # 更新上一时刻的滤波输出值
        prev_rho_err_filtered = rho_err_filtered
        prev_theta_err_filtered = theta_err_filtered

        if line.magnitude()>12:
            linexy=f"[l,{(rho_err_filtered+30)/60:.4f},{theta_err_filtered*2}]"
            uart.write(linexy+'\r\n')
        else:
            linexy=f"[L，{rho_err_filtered+30}，{theta_err_filtered*2}]"
            uart.write(linexy+'\r\n')
        print(linexy)


yellow_threshold = (62, 100, -128, 127, -128, 127)  # 这是黄色的大致颜色范围
#rho 是从图像原点到直线的垂直距离， theta 是直线与垂直轴之间的角度。
def find_lines():
    img = sensor.snapshot().binary([yellow_threshold]).invert().erode(4)
    #获取图像的快照，并将其转换为二值图像，根据之前定义的阈值进行二值化处理。
    line = img.get_regression([(100,100)], robust = True)
    #在二值图像中检测线段，返回一个线段对象line。
    #参数[(100,100)]表示检测线段时使用的区域，robust=True表示使用鲁棒回归算法进行线段拟合。
    if (line): #如果检测到了线段。
        rho_err = abs(line.rho())-img.width()/2
        #计算线段的偏移距离，即线段的rho值减去图像宽度的一半。
        if line.theta()>90: #如果线段的角度大于90度。
            theta_err = line.theta()-180 #计算线段的角度偏差，即线段的角度减去180度。
        else:
            theta_err = line.theta() # 小于等于90度 直接将线段的角度作为角度偏差。
        img.draw_line(line.line(), color = 127) # 在图像上绘制检测到的线段。
        print(rho_err,theta_err) # 打印线段的偏移距离、线段的长度和偏移距离。



w = 1
s = 1
u = 1
v = 1
p = 1
command ='y'
i = 0
prev_time = time.ticks_ms()  # 获取当前的时间（毫秒）
while True:
    clock.tick()  # 更新时钟以跟踪每帧之间的时间
    i += 1
    if i > 9:
        i = 0
    #print(i)
    uart.write(f"{i}\r\n")

    if uart.any():
        receive_data = uart.read()

        try:
            command = receive_data.decode('utf-8').strip()
            print(command)  # 去除可能存在的空白字符

            if command == 'e':
                w = 1
            elif command == 'y':
                s = 1
            elif command == 'c':
                u = 1
            elif command == 'l':
                v = 1

        except UnicodeDecodeError:
            print("接收到的字节不是有效的UTF-8编码")

    if command == 'e':  # 二维码
        if w == 1:
            sensor.set_vflip(False)
            sensor.set_hmirror(False)
            sensor.set_transpose(False)
            sensor.set_framesize(sensor.QVGA)
            sensor.skip_frames(20)
            w = 0
        get_qrcode()
    elif command == 'y':  # 低像素寻找圆
        if s == 1:
            sensor.set_vflip(False)
            sensor.set_hmirror(False)
            sensor.set_transpose(False)
            sensor.set_framesize(sensor.QQVGA)
            sensor.skip_frames(time=10)
            s = 0
        roi = calculate_roi(80, 60, 160, 120)  # QVGA
        get_circles(max_x=160, max_y=120, threshold=2000, r_min=16, r_max=30)
    elif command == 'c':  # 高像素找圆
        if u == 1:
            sensor.set_vflip(False)
            sensor.set_hmirror(False)
            sensor.set_transpose(False)
            sensor.set_framesize(sensor.QVGA)
            sensor.set_auto_gain(False)     #关闭自动增益
            sensor.set_auto_exposure(False)
            sensor.set_auto_whitebal(False)
            sensor.set_pixformat(sensor.RGB565)
            sensor.skip_frames(20)
            u = 0
        # roi = calculate_roi(160, 120, 320, 240)  # HVGA
        # roi = calculate_roi(240, 160, 480, 320)  # HVGA
        get_circles(max_x=320, max_y=240, threshold=2500, r_min=30, r_max=60)
    elif command == 'l':  # 寻线
        if v == 1:
            sensor.set_vflip(True)
            sensor.set_hmirror(False)
            sensor.set_transpose(True)
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QQQVGA)
            sensor.set_auto_gain(False)     #关闭自动增益
            sensor.set_auto_exposure(False)
            sensor.set_auto_whitebal(False)
           # sensor.set_contrast(0)
            sensor.skip_frames(time=10)
            v = 0
        seek_line()  # 进入 seek_line 循环
        #find_lines()
    elif command == 'g':
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(time=10)
        get_color()
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=10)
        get_circles()
    #print("FPS:", clock.fps())  # 打印每秒的帧率
