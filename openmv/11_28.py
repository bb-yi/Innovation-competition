from pyb import LED
import sensor
import image
import time
import pyb
import math
from machine import UART
uart = UART(3, baudrate=115200)

# 高像素时颜色识别使用阈值：
thresholds = [
(7, 99, 3, 42, 3, 26) ,#(38, 63, 21, 51, -5, 39),       # 红色R(38, 52, 21, 52, 0, 31)(29, 54, 9, 39, -9, 22)
(0, 100, -30, -13, 11, 47),#(46, 71, -15, 4, 15, 44),     # 绿色G(46, 71, -15, 4, 15, 44)(49, 63, -28, -10, 6, 43)
(7, 62, -15, 18, -35, -11)                             #(34, 66, 2, 20, -26, -7) # 蓝色B(34, 66, 2, 20, -26, -7) (33, 52, -4, 11, -23, -4)
]
# c放置圆环颜色识别使用阈值：
thresholds_find_cir = [
              (7, 99, 3, 42, 3, 26) ,#红
              (11, 82, -17, 1, 2, 28),#绿
              (15, 95, -6, 11, -30, -10)#蓝
              ]

# y转盘处色块识别使用阈值：thresholds2
# d堆垛色块识别别使用阈值：thresholds2
# thresholds1没用上的
thresholds1=[(21, 57, 9, 57, 8, 38),
         (21, 57, -52, -17, 14, 38) ,
          (21, 50, -18, 11, -46, -14)
]
thresholds2=[(0, 100, 19, 68, 10, 56),
             (0, 100, -75, -35, 0, 36),
             (0, 100, -13, 12, -66, -23)
]

# l寻线参数
kernel = [-1, 0,  1,
           -2,  1,  2,
            -1,  0,  1]
clock = time.clock()

def draw_line_from_point_and_angle(x, y, angle, img):
    # 将角度转换为弧度
    angle_rad = math.radians(angle)
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
def calculate_score(magnitude, mid_x, length, weight_magnitude=0.5, weight_mid_x=0.3, weight_length=0.2):
    # 可以根据实际情况调整权重
    return weight_magnitude * magnitude + weight_mid_x * mid_x + weight_length * length
def seek_line_2():
    clock.tick()
    img = sensor.snapshot().lens_corr(1.6)
    #img.gaussian(1)

    img.morph(1, kernel)
    #img.gaussian(1)
    # 获取图像的宽度和高度
    width = img.width()
    height = img.height()

    # 查找图像中的所有直线
    lines = img.find_lines(threshold = 2200, theta_margin = 25, rho_margin = 25)
    if lines:
        max_magnitude = 0
        max_line = None
        max_score = 0
        max_score_details = (0, 0, 0)  # 用于存储最大评分对应的magnitude, mid_x,
        # 遍历所有直线，找到幅度最大的直线
        for line in lines:

            line_angle1 = line.theta() if line.theta()<90 else line.theta()-180
            if abs(line_angle1)<25:
                img.draw_line(line.line(), color=(255, 0, 0))
                magnitude = line.magnitude()
                mid_x = (line.x1() + line.x2()) / 2
                length = line.length()
                score = calculate_score(magnitude,width-mid_x,length,0.2, 8.0, 0.1)

            #print(f"Magnitude: {magnitude}, Mid X: {mid_x}, Length: {length}, Score: {score}")

                if score > max_score:
                    max_score = score
                    max_line = line
                    max_score_details = (magnitude, mid_x, length)
        # 如果找到了最大幅度的直线，绘制它
        if max_line:
            line_angle = max_line.theta() if max_line.theta()<90 else max_line.theta()-180
            line_mid_x = (max_line.x1() + max_line.x2()) / 2
            line_mid_y = (max_line.y1() + max_line.y2()) / 2

            global last_angle,last_line_mid_x,last_line_mid_y
            pos_alpha = 0.2
            if last_angle is not None:
                line_angle = low_pass_filter(line_angle, last_angle, alpha=0.5)
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
        else:
            send_message = "None"


    else:
        send_message = "None"
    uart.write((send_message + '\r\n').encode())  # 转换为字节数据
    print(send_message)  # 打印不需要追加 '\r\n'
    # 输出当前帧率
    #print(clock.fps())







t=0

def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        area = blob.pixels()
        if area > max_size:
            max_blob = blob
            max_size = area
    return max_blob, max_size

def get_color():
    img = sensor.snapshot()
    width = img.width()
    height = img.height()

    # 存储每个颜色的最大色块及其面积
    max_blobs = []

    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    for i, thresholds in enumerate(thresholds2):
        blobs = img.find_blobs([thresholds])
        if blobs:
            max_blob, _ = find_max(blobs)

            if max_blob.pixels() >4000:
                print(max_blob.pixels())

                max_blobs.append((max_blob, colors[i]))
            else:
                message="None"
        else:
            message="None"

    # 如果没有找到任何色块，则直接返回
    if not max_blobs:
        message="None"
        print(message)

        uart.write(message.encode())

        return

    # 从所有颜色的最大色块中再次选出面积最大的一个
    final_max_blob, _ = find_max([blob[0] for blob in max_blobs])

    # 根据最终选择的色块来确定颜色
    t = 0
    if final_max_blob is not None:
        for blob, color in max_blobs:
            if blob[0] == final_max_blob[0]:
                t = [1, 2, 3][colors.index(color)]  # 确定颜色标识
                break

    # 绘制矩形和十字标记
    img.draw_rectangle(final_max_blob[0:4])
    img.draw_cross((final_max_blob[5], final_max_blob[6]), size=5, color=colors[t-1])

    # 发送消息
    message = f"[y,{final_max_blob[5]/width:.4f},{final_max_blob[6]/height:.4f},{t}]"
    print(message)
    uart.write(message.encode())

def get_color2():
    img = sensor.snapshot()

    width = img.width()
    height = img.height()

    # 存储每个颜色的最大色块及其面积
    max_blobs = []

    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    for i, thresholds in enumerate(thresholds2):
        blobs = img.find_blobs([thresholds])
        if blobs:
            max_blob, _ = find_max(blobs)

            if max_blob.pixels() >2000:
                print(max_blob.pixels())

                max_blobs.append((max_blob, colors[i]))
            else:
                message="None"
        else:
            message="None"

    # 如果没有找到任何色块，则直接返回
    if not max_blobs:
        message="None"
        print(message)

        uart.write(message.encode())

        return

    # 从所有颜色的最大色块中再次选出面积最大的一个
    final_max_blob, _ = find_max([blob[0] for blob in max_blobs])

    # 根据最终选择的色块来确定颜色
    t = 0
    if final_max_blob is not None:
        for blob, color in max_blobs:
            if blob[0] == final_max_blob[0]:
                t = [1, 2, 3][colors.index(color)]  # 确定颜色标识
                break

    # 绘制矩形和十字标记
    img.draw_rectangle(final_max_blob[0:4])
    img.draw_cross((final_max_blob[5], final_max_blob[6]), size=5, color=colors[t-1])

    # 发送消息
    message = f"[d,{final_max_blob[5]/width:.4f},{final_max_blob[6]/height:.4f},{t}]"
    print(message)
    uart.write(message.encode())


def determine_color(color, threshold=40):
    #print(color)
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

def calculate_roi(center_x, center_y, width, height):
    # 计算矩形的左上角坐标
    x = int(center_x - width / 2)
    y = int(center_y - height / 2)
    #返回 ROI，格式为 (x, y, w, h)
    #就是做中心矩形
    return (x, y, width, height)

thresholds_find_cir = [
              (7, 99, 3, 42, 3, 26) ,#红
              (11, 82, -17, 1, 2, 28),#绿
              (15, 95, -6, 11, -30, -10)#蓝
              ]
last_x, last_y = None, None
def find_circle():
    clock.tick()
    target_blob = None
    message = "None"
    target_dis = 999999
    img = sensor.snapshot().lens_corr(1.6)  # 获取当前图像
    width = sensor.width()
    height = sensor.height()
    ROI =calculate_roi((width / 2),(height / 2),int(width*0.8),int(height*0.8))
    # 使用阈值列表，查找多个颜色的色块
    #下一行的thresholds本来是thresholds_find_cir的
    blobs = img.find_blobs(thresholds, area_threshold=10, merge=False)  # 查找符合阈值范围的色块
    if blobs:
        for blob in blobs:
            if blob.density() > 0.2 and blob.area()>500:
                # 绘制检测到的色块的矩形框和中心点
                dis = (blob.cy() - (height / 2)) ** 2 + (blob.cx() - (width / 2)) ** 2
                if dis < target_dis:
                    target_dis = dis
                    target_blob = blob
                img.draw_rectangle(blob.rect(), color=(0, 255, 0))  # 绘制矩形框，绿色

    if target_blob is not None:
        img.draw_rectangle(target_blob.rect(), color=(255, 0, 0))  # 绘制矩形框，红色
        img.draw_cross(target_blob.cx(), target_blob.cy(), size=3, color=(255, 255, 255))  # 绘制交叉线，白色

        # 根据色块的区域类型选择二值化阈值
        target_color = target_blob.code() if target_blob.code() != 4 else 3
        if target_color in [1,2,3]:
            #print(target_color)
            if 1:
                img.binary([thresholds[target_color - 1]])  # 阈值范围
                img.midpoint(1, bias=0.5)#中点滤波
                threshold = [30]  # 单一阈值作为列表传递
                img.binary([threshold])
                img.mean(1)#均值滤波
                # 查找圆形，调整下面的参数以适应你的场景
                circles = img.find_circles(roi=ROI,threshold=4500,  r_min=15, r_max=20, r_step=2)
                # 绘制找到的圆
                if circles:
                    max_magnitude = 0
                    target_circle = None
                    for circle in circles:
                        if circle.magnitude() >max_magnitude:
                            target_circle = circle
                    pos_alpha = 0.1
                    if last_x is not None:
                        current_x = low_pass_filter(target_circle.x(),last_x,pos_alpha)
                    else:
                        current_x = target_circle.x()
                    if last_y is not None:
                        current_y = low_pass_filter(target_circle.y(),last_y,pos_alpha)
                    else:
                        current_y = target_circle.y()
                    img.draw_circle(current_x, current_y, target_circle.r(), color=(255, 0, 0))  # 绘制红色圆
                    img.draw_cross(current_x, current_y, color=(0, 255, 0))  # 绘制绿色交叉线，圆心位置
                    message = f"[y,{current_x/width:.4f},{current_y/height:.4f},{target_color}]"  # 消息内容
    print(message)
    uart.write(message.encode())  # 发送字节串
    #[y,0.5083,0.5187,3]



s = 1
u = 1
v = 1
m = 1

command = 'c'
i = 0
last_command= 'l'
while True:
    #i += 1
    #if i > 9:
    #    i = 0
    #print(i)
    #uart.write(f"{i}\r\n")
    if uart.any():
        receive_data = uart.read()

        try:
            last_command=command
            command = receive_data.decode('utf-8').strip()
            print(command)
            if command == 'y':
                s = 1
            elif command == 'c':
                u = 1
            elif command == 'l':
                v=1
            elif command == 'd':
                m=1

        except UnicodeDecodeError:
            print("接收到的字节不是有效的UTF-8编码")

    elif command == 'y':  # 低像素寻找圆
        if s == 1 and last_command != 'y':
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QVGA)
            sensor.set_auto_exposure(False,exposure_us=70000 )
            sensor.set_auto_gain(False, gain_db=14)
            sensor.set_auto_whitebal(False, rgb_gain_db =  (63.2751, 60.206, 61.5473))
            sensor.skip_frames(20)
            s = 0
        #roi= calculate_roi(80, 60, 160, 120)
        #get_circles(roi=roi, threshold=1600, r_min=14, r_max=24)
        #get_circles(roi=roi, threshold=2500, r_min=24, r_max=34)
        get_color()


        #print(sensor.set_auto_whitebal(False))

    elif command == 'c':  # 高像素找圆
        if u == 1 and last_command != 'c':
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.HQVGA)
            sensor.set_auto_exposure(False,exposure_us=70000 )
            sensor.set_auto_gain(False, gain_db=14)
            sensor.set_auto_whitebal(False, rgb_gain_db =  (63.2751, 60.206, 61.5473))
            sensor.skip_frames(20)
            u = 0
        find_circle()

    elif command == 'l':
        if v == 1 and last_command != 'l':
            sensor.reset()
            sensor.set_vflip(True)
            sensor.set_hmirror(False)
            sensor.set_transpose(True)
            sensor.set_auto_gain(False, gain_db=12)
            sensor.set_auto_whitebal(False, rgb_gain_db =  (63.2751, 60.206, 61.5473))
            sensor.set_auto_exposure(False,exposure_us=60000 )
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.HQVGA) #160  120 QQVGA
            sensor.skip_frames(20)
            v = 0
        #seek_line()
        seek_line_2()
    elif command == 'd':  # 低像素寻找圆
        if m == 1 and last_command != 'd':
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QVGA)
            sensor.set_auto_exposure(False,exposure_us=70000 )
            sensor.set_auto_gain(False, gain_db=14)
            sensor.set_auto_whitebal(False, rgb_gain_db =  (63.2751, 60.206, 61.5473))
            sensor.skip_frames(20)
            m = 0
        get_color2()
    print(command,last_command)

