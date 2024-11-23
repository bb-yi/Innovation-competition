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
   (38, 63, 21, 51, -5, 39),       # 红色R(38, 52, 21, 52, 0, 31)(29, 54, 9, 39, -9, 22)
(46, 71, -15, 4, 15, 44),     # 绿色G(46, 71, -15, 4, 15, 44)(49, 63, -28, -10, 6, 43)
(34, 66, 2, 20, -26, -7) # 蓝色B(34, 66, 2, 20, -26, -7) (33, 52, -4, 11, -23, -4)
]
#补光灯蓝色(25, 63, -9, 30, -42, -9) 绿色 (52, 67, -17, 6, -7, 43)

kernel_size = 1 # 3x3==1, 5x5==2, 7x7==3, etc.

kernel1 = [-2, -1,  0, \
          -1,  1,  1, \
           0,  1,  2]
kernel2 = [0, -1,  -2, \
         1,  1,  -1, \
          2,  1,  0]

enable_lens_corr = True
min_degree = 0
max_degree = 179
angle1=None
angle2=None
distance1=None
distance2=None
distance_output=None
last_distance1=30
last_distance2=30
max_dev_distance=20
xishu=0.7
def seek_line():
    global angle1, angle2, last_distance1, last_distance2, distance_output, angle_outpu
    img = sensor.snapshot()
    img.morph(kernel_size, kernel1)
    if enable_lens_corr: img.lens_corr(1.8)
    l1s = img.find_lines(threshold = 1500, theta_margin = 25, rho_margin = 25)
    if l1s :
        for l1 in l1s:
            if (min_degree <= l1.theta()) and (l1.theta() <= max_degree):

                img.draw_line(l1.line(), color = (255, 0, 0))
                new_distance1 = int(((l1.x1() + l1.x2()) / 2) * xishu + last_distance1 * (1 - xishu))

                # 检查距离偏差
                if abs(new_distance1 - last_distance1) <= max_dev_distance:
                    distance1 = new_distance1
                    last_distance1 = distance1
                angle1=l1.theta()
                if angle1>90:
                    angle1=angle1-180


                #print('越小越接近边界',(l1.x1()+l1.x2())/2)
                #print('红线角度',angle1)
    else:
        angle1=None
        distance_output=None

    img2 = sensor.snapshot()
    img2.morph(kernel_size, kernel2)
    if enable_lens_corr: img2.lens_corr(1.8)
    l2s=img2.find_lines(threshold = 1500, theta_margin = 25, rho_margin = 25)
    if l2s:
        for l2 in l2s:
            if (min_degree <= l2.theta()) and (l2.theta() <= max_degree):
                img2.draw_line(l2.line(), color = (0, 255, 0))
                new_distance2 = int(((l2.x1() + l2.x2()) / 2) * xishu + last_distance2 * (1 - xishu))

                # 检查距离偏差
                if abs(new_distance2 - last_distance2) <= max_dev_distance:
                    distance2 = new_distance2
                    last_distance2 = distance2

                angle2=l2.theta()
                if angle2>90:
                    angle2=angle2-180

                #print('越小越接近边界',(l2.x1()+l2.x2())/2)
                #print('绿线角度',angle2)
    else:
        angle2=None
        distance_output=None


    if angle1 is not None and angle2 is not None:
        angle_output = (angle1 + angle2) / 2
        distance_output = (distance1 + distance2) / 2

    elif angle1 is not None:
        angle_output = angle1
        distance_output = distance1

    elif angle2 is not None:
        distance_output = distance2
        angle_output = angle2
    else:
        angle_output=None
        distance_output=None



    linexy = f"[l,{distance_output},{angle_output}]"
    uart.write(linexy+'\r\n')
    print(linexy)

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
    img = sensor.snapshot().lens_corr(1.2)
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
                score = calculate_score(magnitude,width-mid_x,length,0.2, 5.0, 0.1)

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
            pos_alpha = 0.15
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



def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob


thresholds1=[(24, 56, 11, 56, 19, 49),
             (24, 56, -47, -18, 9, 39),
          (24, 56, -12, 17, -48, -19)
]
t=0
def get_color():
    img=sensor.snapshot()
    width = img.width()
    height = img.height()
    red_blobs = img.find_blobs([thresholds1[0]])
    if red_blobs:
        max_blob = find_max(red_blobs)
        img.draw_rectangle(max_blob[0:4])
        img.draw_cross((max_blob[5], max_blob[6]), size=5, color=(255, 0, 0))
        t=1
        message = f"[y,{max_blob[5]/width:.4f},{max_blob[6]/height:.4f},{t}]"  # 消息内容
        print(message)
        uart.write(message.encode())
    blue_blobs = img.find_blobs([thresholds1[1]])
    if blue_blobs:
        max_blob = find_max(blue_blobs)
        img.draw_rectangle(max_blob[0:4])
        img.draw_cross((max_blob[5], max_blob[6]), size=5, color=(0, 255, 0))
        t=2
        message = f"[y,{max_blob[5]/width:.4f},{max_blob[6]/height:.4f},{t}]"  # 消息内容
        print(message)
        uart.write(message.encode())
    green_blobs = img.find_blobs([thresholds1[2]])
    if green_blobs:
        max_blob = find_max(green_blobs)
        img.draw_rectangle(max_blob[0:4])
        img.draw_cross((max_blob[5], max_blob[6]), size=5, color=(0, 0, 255))
        t=3
        message = f"[y,{max_blob[5]/width:.4f},{max_blob[6]/height:.4f},{t}]"  # 消息内容
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


# 初始化全局变量
last_x, last_y = None, None
max_deviation = 60  # 允许的最大偏差值（可以根据需求调整）
last_Radius = 10
# roi = calculate_roi(80, 60, 160, 120)  # QQVGA
# roi = calculate_roi(160, 120, 320, 240)#QVGA
# roi = calculate_roi(320, 240, 640, 480)#VGA
#roi = calculate_roi(240, 160, 360, 240)  # HVGA
color = (0,0,0)

def get_circles(roi, threshold, r_min, r_max):
    global last_x, last_y,color
    img = sensor.snapshot()
    width = img.width()
    height = img.height()
    circles = img.find_circles(roi=roi, threshold=threshold, x_margin=10, y_margin=10, r_margin=10, r_min=r_min, r_max=r_max, r_step=1)

    if circles:
        t= circles[0]  #获取第一个圆
        pos_alpha = 0.3
        if last_x is not None:
            current_x = low_pass_filter(t.x(),last_x,pos_alpha)
        else:
            current_x = t.x()
        if last_y is not None:
            current_y = low_pass_filter(t.y(),last_y,pos_alpha)
        else:
            current_y = t.y()
        Radius = t.r()
        roi = calculate_roi(current_x, current_y, 80, 80)
        img.draw_rectangle(roi,(0,0,255))
        # 计算与上一次圆心位置和半径的偏差
        #deviation = math.sqrt((current_x - last_x) ** 2 + (current_y - last_y) ** 2)
        #radius_deviation = abs(Radius - last_Radius)
        # 如果偏差过大，忽略本次更新
        #if deviation > max_deviation:
        #    print(f"圆心或半径变化过大，忽略本次结果。偏差: {deviation}, 半径偏差: {radius_deviation}")
        #    last_x = current_x
        #    last_y = current_y
        #    return  # 跳过当前结果
        if current_x>width:
            current_x=width-1
        if current_y>height:
            current_y=height-1
        print(current_x, current_y,Radius)
        color = img.get_pixel(int(current_x), int(current_y))
        #print(color)

        # 输出圆心位置及颜色信息
        message = f"[y,{current_x/width:.4f},{current_y/height:.4f},{determine_color(color,10)}]"  # 消息内容

        # 画出圆心和圆
        img.draw_circle(int(current_x), int(current_y), Radius, color=(255, 0, 0))
        img.draw_cross(int(current_x), int(current_y), size=5, color=(255, 0, 0))
        # 更新上一次的圆心位置和半径
        last_x, last_y = current_x, current_y
        last_Radius = Radius
    else:
        g=20
        #roi = calculate_roi(last_x, last_y, 80+g, 80+g)
        roi= calculate_roi(80, 60, 160, 120)

        message = "NONE"
        img.draw_rectangle(roi, color=(0, 255, 0), thickness=1)  # 绘制绿色矩形表示 ROI
    print(message)
    uart.write(message.encode())  # 发送字节串




last_pos_x =None
last_pos_y =None
def get_circles_2(roi,threshold, r_min, r_max):
    global circle3
    # 确保 circle3 已经初始化为 [0, 0, 0]
    if 'circle3' not in globals() or not isinstance(circle3, list) or len(circle3) != 3:
        circle3 = [0, 0, 0]
    img = sensor.snapshot()
    width = img.width()
    height = img.height()
    img.draw_rectangle(ROI2, color=(0, 0, 0))
    circles = img.find_circles(roi=roi,threshold=threshold, x_margin=10, y_margin=10, r_margin=10, r_min=r_min, r_max=r_max, r_step=3)

    if circles:
        max_circle = find_max_circle(circles)
        global last_pos_x,last_pos_y
        alpha = 0.5
        if max_circle is not None:
            if last_pos_x is not None:
                pos_x = low_pass_filter(max_circle.x(),last_pos_x,alpha)
            else:
                pos_x = max_circle.x()
            if last_pos_y is not None:
                pos_y = low_pass_filter(max_circle.y(),last_pos_y,alpha)
            else:
                pos_y = max_circle.y()
            last_pos_x = pos_x
            last_pos_y = pos_y
            img.draw_circle(int(pos_x), int(pos_y), max_circle.r(), color=(255, 0, 0))
            img.draw_cross((max_circle.x(), max_circle.y()), size=5, color=(255, 0, 0))
        X=max_circle.x()/320
        Y=max_circle.y()/320
        #print(f"X: {max_circle.x()}, Y: {max_circle.y()}, R: {max_circle.r()}")
        #roi_color=calculate_roi(max_circle.x(), max_circle.y()+0.7*max_circle.r(),round(1.3*max_circle.r()),round(0.75*max_circle.r()))
        roi_color=calculate_roi(max_circle.x()+0.7*max_circle.r(), max_circle.y(),round(0.75*max_circle.r()),round(1.3*max_circle.r()))

        img.draw_rectangle(roi_color, color=(0, 0, 0))
        color_found = False
        for i, threshold in enumerate(thresholds):
            color_name = ["1", "2", "3"][i]
            blobs = img.find_blobs([threshold], roi=roi_color)
            if blobs:
                max_blob = find_max_blob(blobs)
                if max_blob is not None:
                    img.draw_rectangle(max_blob.rect(), color=(255, 0, 0))
                    img.draw_cross((max_blob.cx(), max_blob.cy()), size=5, color=(255, 0, 0))
                    message = f"[y,{max_circle.x()/width:.4f},{max_circle.y()/height:.4f},{color_name}]"
                    print(message)
                    uart.write(message.encode())
                    color_found = True
                    break
        if not color_found:
            message = f"[y,{max_circle.x()/width:.4f},{max_circle.y()/height:.4f},0]"
            print(message)
            uart.write(message.encode())
    else:
        message = "None"

        uart.write(message.encode())


def find_max_circle(circles):
    """ 从给定的圆列表中找到半径最大的圆。 """
    if not circles:
        return None
    max_circle = max(circles, key=lambda c: c.r())
    return max_circle

def find_max_blob(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob


s = 1
u = 1
v = 1
m = 1

command = 'g'
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
            print(command)
            if command == 'y':
                s = 1
            elif command == 'c':
                u = 1
            elif command == 'l':
                v=1
            elif command == 'g':
                m=1

        except UnicodeDecodeError:
            print("接收到的字节不是有效的UTF-8编码")
    elif command == 'y':  # 低像素寻找圆
        if s == 1:
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QQVGA)
            sensor.set_auto_exposure(False,exposure_us=60000 )
            sensor.set_auto_gain(False, gain_db=13)
            sensor.set_auto_whitebal(False, rgb_gain_db =  (63.2751, 60.206, 61.5473))
            sensor.skip_frames(10)
            s = 0
        roi= calculate_roi(80, 60, 160, 120)
        #get_circles(roi=roi, threshold=1600, r_min=14, r_max=24)
        get_circles(roi=roi, threshold=2500, r_min=24, r_max=34
                    )

        #print(sensor.set_auto_whitebal(False))

    elif command == 'c':  # 高像素找圆
        if u == 1:
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QVGA)
            sensor.set_auto_exposure(False,exposure_us=60000 )
            sensor.set_auto_gain(False, gain_db=14)
            sensor.set_auto_whitebal(False, rgb_gain_db =  (63.2751, 60.206, 61.5473))
            sensor.skip_frames(20)
            u = 0
        ROI2 = calculate_roi(160,120,250,200)
        get_circles_2(roi=ROI2, threshold=2350, r_min=43, r_max=52)
    elif command == 'l':
        if v == 1:
            sensor.reset()
            sensor.set_vflip(True)
            sensor.set_hmirror(False)
            sensor.set_transpose(True)
            sensor.set_auto_gain(False, gain_db=12)
            sensor.set_auto_whitebal(False, rgb_gain_db =  (63.2751, 60.206, 61.5473))
            sensor.set_auto_exposure(False,exposure_us=60000 )
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.HQVGA) #160  120 QQVGA
            sensor.skip_frames(time = 20)
            v = 0
        #seek_line()
        seek_line_2()
    elif command == 'g':  # 低像素寻找圆
        if m == 1:
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QQVGA)
            sensor.set_auto_exposure(False,exposure_us=60000 )
            sensor.set_auto_gain(False, gain_db=13)
            sensor.set_auto_whitebal(False, rgb_gain_db =  (63.2751, 60.206, 61.5473))
            sensor.skip_frames(10)
            m = 0
        get_color()


