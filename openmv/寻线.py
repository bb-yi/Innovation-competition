# Untitled - By: li - Sat Nov 16 2024

import sensor, image, time
import math  # 导入 math 库
from machine import UART
uart = UART(3, baudrate=115200)
sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(False)
sensor.set_transpose(True)
sensor.set_auto_gain(False, gain_db=30)
sensor.set_auto_whitebal(False, rgb_gain_db =  (63.2751, 60.206, 61.5473))
sensor.set_auto_exposure(False,exposure_us=100000 )
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.HQVGA) #160  120 QQVGA
sensor.skip_frames(time = 20)

kernel1 = [-1, 0,  1,
           -2,  0,  2,
            -1,  0,  1]
clock = time.clock()

def draw_line_from_point_and_angle(x, y, angle, img):
    # 将角度转换为弧度
    angle_rad = math.radians(angle)
    #print(angle)
    # 如果角度为 90 或 270，表示垂直线，直接处理
    if angle == 90 or angle == 270:
        # 垂直线：x 坐标不变，y 坐标变化
        x0 = x
        x1 = x
        # 线段延伸到图像的上下边界
        y0 = 0
        y1 = img.height()
    else:
        # 计算直线的斜率
        slope = math.tan(angle_rad)
        # 计算上端点（y=0时的x坐标）
        y0 = 0
        x0 = x - (y - y0) / slope  # 通过点斜式计算x0
        # 计算下端点（y=img.height()时的x坐标）
        y1 = img.height()
        x1 = x - (y - y1) / slope  # 通过点斜式计算x1
    # 使用 img.draw_line() 绘制线段
    img.draw_line(int(x0), int(y0), int(x1), int(y1), color=(0, 255, 0), thickness=2)  # 这里绘制的是红色线条
    # 输出绘制的直线信息
    #print(f"Line from ({x0}, {y0}) to ({x1}, {y1}) with angle {angle} degrees")


# 初始化角度值
last_angle = None
last_line_mid_x = None
last_line_mid_y = None
def low_pass_filter(new_value, last_value, alpha=0.5):
    # 一阶低通滤波，alpha是滤波系数，范围通常在0到1之间
    return alpha * new_value + (1 - alpha) * last_value

def seek_line():
    clock.tick()
    img = sensor.snapshot().lens_corr(1.2)
    img.gaussian(1)

    img.morph(1, kernel1)

    # 获取图像的宽度和高度
    width = img.width()
    height = img.height()

    # 查找图像中的所有直线
    lines = img.find_lines(threshold = 2200, theta_margin = 25, rho_margin = 25)
    if lines:
        max_magnitude = 0
        max_line = None
        # 遍历所有直线，找到幅度最大的直线
        for line in lines:
            #img.draw_line(line.line(), color=(255, 0, 0))
            magnitude = line.magnitude()
            #magnitude = line.length()
            if magnitude > max_magnitude:
                max_magnitude = magnitude
                max_line = line
        # 如果找到了最大幅度的直线，绘制它
        if max_line:
            #img.draw_line(max_line.line(), color=(0, 255, 0))
            line_angle = max_line.theta() if max_line.theta()<90 else max_line.theta()-180
            line_mid_x = (max_line.x1() + max_line.x2()) / 2
            line_mid_y = (max_line.y1() + max_line.y2()) / 2

            global last_angle,last_line_mid_x,last_line_mid_y
            pos_alpha = 0.2
            if last_angle is not None:
                line_angle = low_pass_filter(line_angle, last_angle, alpha=0.3)
            if last_line_mid_x is not None:
                line_mid_x = low_pass_filter(line_mid_x, last_line_mid_x, alpha=pos_alpha)
            if last_line_mid_y is not None:
                line_mid_y = low_pass_filter(line_mid_y, last_line_mid_y, alpha=pos_alpha)
            last_angle = line_angle
            last_line_mid_x = line_mid_x
            last_line_mid_y = line_mid_y
            draw_line_from_point_and_angle(line_mid_x, line_mid_y, line_angle+90, img)
            #print("角度:",line_angle,"X:",line_mid_x,"Y:",line_mid_y)
            send_message = f"[l,{line_mid_x:.2f},{line_angle:.2f}]"
            uart.write((send_message + '\r\n').encode())  # 转换为字节数据
            #print(send_message)  # 打印不需要追加 '\r\n'
    else:
        print(None)
        #print(avg_angle,avg_mid_pos)
        #img.draw_line((line_x1, line_y1, line_x2, line_y2), color=(0, 255, 0))  # 绘制绿色的平均直线

    # 输出当前帧率
    #print(clock.fps())




while(True):
    seek_line()
