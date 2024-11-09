from pyb import LED
import sensor
import image
import time
import pyb
import math
from machine import UART
uart = UART(3, baudrate=115200)
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

# 巡线的二值化所用阈值
THRESHOLD = (46, 75, -12, 12, -4, 20)  # 白色纸面灰色线识别阈值
# (62, 82, -15, 4, 9, 27)黄色边框条
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
            if len(qrcode) != 0:
                pyb.delay(100)
                red_led.on()
                pyb.delay(100)
                red_led.off()
                pyb.delay(100)
                red_led.on()
                pyb.delay(100)
                red_led.off()

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

def get_dominant_color_in_roi(roi):
    img = sensor.snapshot()
    #red_threshold = (24, 58, 11, 81, -52, 77)# 红色阈值
    #reen_threshold = (42, 60, -34, -4, -1, 108)  # 绿色阈值
    #blue_threshold = (18, 50, -5, 13, -39, -8)      # 蓝色阈值
    red_threshold = (0, 0, -128, -128, -128, -128)# 红色阈值
    green_threshold = (0, 0, -128, -128, -128, -128)  # 绿色阈值
    blue_threshold = (0, 0, -128, -128, -128, -128)      # 蓝色阈值

    # 查找红、绿、蓝、白色块
    red_blobs = img.find_blobs([red_threshold], roi=roi, pixels_threshold=10, area_threshold=10, merge=True)
    green_blobs = img.find_blobs([green_threshold], roi=roi, pixels_threshold=10, area_threshold=10, merge=True)
    blue_blobs = img.find_blobs([blue_threshold], roi=roi, pixels_threshold=10, area_threshold=10, merge=True)

    red_area, green_area, blue_area = 0, 0, 0

    # 计算红色块面积
    for blob in red_blobs:
        red_area += blob.area()

    # 计算绿色块面积
    for blob in green_blobs:
        green_area += blob.area()

    # 计算蓝色块面积
    for blob in blue_blobs:
        blue_area += blob.area()
    #print({red_area},{green_area},{blue_area})

    # 判断占比最大的颜色
    if red_area > green_area and red_area > blue_area:
        return (255,0,0)
    elif green_area > red_area and green_area > blue_area:
        return (0,255,0)
    elif blue_area > red_area and blue_area > green_area:
        return (0,0,255)
    else:
        return (100,100,100)  # 无明显主颜色

def calculate_roi(center_x, center_y, width, height):
    # 计算矩形的左上角坐标
    x = int(center_x - width / 2)
    y = int(center_y - height / 2)

    # 返回 ROI，格式为 (x, y, w, h)
    return (x, y, width, height)


# 初始化全局变量
last_x, last_y = 240, 160
max_deviation = 80  # 允许的最大偏差值（可以根据需求调整）
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
        c = circles[0]  # 获取第一个圆
        pos_alpha = 0.7
        current_x = int(c.x() * pos_alpha + last_x * (1 - pos_alpha))
        current_y = int(c.y() * pos_alpha + last_y * (1 - pos_alpha))
        Radius = int(c.r() * 0.2 + last_Radius * 0.8)

        # 计算与上一次圆心位置和半径的偏差
        deviation = math.sqrt((current_x - last_x) ** 2 + (current_y - last_y) ** 2)
        radius_deviation = abs(Radius - last_Radius)

        # 如果偏差过大，忽略本次更新
        if deviation > max_deviation:
            print(f"圆心或半径变化过大，忽略本次结果。偏差: {deviation}, 半径偏差: {radius_deviation}")
            last_x = current_x
            last_y = current_y
            return  # 跳过当前结果
        if (max_x != 160):
            roi = calculate_roi(current_x, current_y, 160, 160)
        if (max_x == 160):
            color = img.get_pixel(current_x, current_y)
        else:
            color = get_dominant_color_in_roi(roi)
            #color =(0,0,0)


        # 输出圆心位置及颜色信息
        message = f"[y,{current_x/width:.4f},{current_y/height:.4f},{determine_color(color,10)}]"  # 消息内容

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
            roi = calculate_roi(240, 160, 360, 240)  # HVGA

        message = "NONE"
    img.draw_rectangle(roi, color=(0, 255, 0), thickness=1)  # 绘制绿色矩形表示 ROI
    print(message)
    uart.write(message.encode())  # 发送字节串


def find_max(circles):
    """ 从给定的圆列表中找到半径最大的圆。 """
    if not circles:
        return None

    max_circle = max(circles, key=lambda c: c.r())
    return max_circle


# dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd

# theta代表返回线段的角度, rho代表偏移的距离
# rho_err 左右偏移偏差值 即小车越接近为负数，远离为正数，正中间为0
# theta_err 角度偏差值 向左偏为负数，向右偏为正数，正中间为0


def seek_line():
    img = sensor.snapshot()
    print(img.width())
    img.binary([THRESHOLD])
    line = img.get_regression([(100, 100)], robust=True)
    if (line):
        rho_err = abs(line.rho())-img.width()/2
        if line.theta() > 90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color=127)
        print(rho_err, theta_err)
        if line.magnitude() > 12:
            print('好')
            linexy = f"[l,{(rho_err+30)/60:.4f},{theta_err}]"
            uart.write(linexy+'\r\n')
        else:
            print('不好')
            linexy = f"[L，{rho_err+30}，{theta_err}]"
            uart.write(linexy+'\r\n')


w = 1
s = 1
u = 1
v = 1
p = 1
command = 'y'
i = 0
while True:

    i += 1
    if i > 9:
        i = 0
    #print(i)
    #uart.write(f"{i}\r\n")

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
            sensor.set_auto_whitebal(False)
            sensor.skip_frames(time=10)

            s = 0
        roi = calculate_roi(80, 60, 160, 120)  # QVGA
        get_circles(max_x=160, max_y=120, threshold=1500, r_min=17, r_max=30)
    elif command == 'c':  # 高像素找圆
        if u == 1:
            sensor.set_vflip(False)
            sensor.set_hmirror(False)
            sensor.set_transpose(False)
            sensor.set_framesize(sensor.HVGA)
            sensor.set_auto_whitebal(False)
            sensor.skip_frames(20)
            u = 0
        # roi = calculate_roi(240, 160, 480, 320)  # HVGA
        get_circles(max_x=480, max_y=320, threshold=2000, r_min=50, r_max=70)
    elif command == 'l':  # 寻线
        if v == 1:
            sensor.set_vflip(True)
            sensor.set_hmirror(False)
            sensor.set_transpose(True)
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.HQQVGA)
            sensor.set_auto_whitebal(False)
            sensor.skip_frames(time=10)
            v = 0
        seek_line()  # 进入 seek_line 循环
    elif command == 'g':
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(time=10)
        get_color()
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=10)
        get_circles()
