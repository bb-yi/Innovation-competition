from pyb import LED
import sensor
import image
import time
import pyb
import math
from machine import UART
uart = UART(3, baudrate=115200)

# 巡线的二值化所用阈值
THRESHOLD = (43, 74, -8, 9, 0, 14)
# (62, 82, -15, 4, 9, 27)黄色边框条

# 高像素时颜色识别使用阈值：
thresholds = [
    (45, 63, 8, 22, 0, 18),       # 红色R
    (59, 62, -18, -9, 2, 47),     # 绿色G
    (46, 60, -16, -1, -16, -3)   # 蓝色B
]




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




yellow_threshold = (43, 74, -8, 9, 0, 14) # 这是黄色的大致颜色范围(65, 69, -13, 7, 1, 10)
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









def determine_color(color, threshold=40):
    print(color)
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
last_x, last_y = 240, 160
max_deviation = 60  # 允许的最大偏差值（可以根据需求调整）
last_Radius = 10
# roi = calculate_roi(80, 60, 160, 120)  # QQVGA
# roi = calculate_roi(160, 120, 320, 240)#QVGA
# roi = calculate_roi(320, 240, 640, 480)#VGA
#roi = calculate_roi(240, 160, 360, 240)  # HVGA
color = (0,0,0)

def get_circles(roi, threshold, r_min, r_max):
    global last_x, last_y,last_Radius,color
    img = sensor.snapshot()
    width = img.width()
    height = img.height()
    circles = img.find_circles(roi=roi, threshold=threshold, x_margin=10, y_margin=10, r_margin=10, r_min=r_min, r_max=r_max, r_step=1)

    if circles:
        t= circles[0]  #获取第一个圆
        pos_alpha = 0.3
        current_x = int(t.x() * pos_alpha + last_x * (1 - pos_alpha))
        current_y = int(t.y() * pos_alpha + last_y * (1 - pos_alpha))
        Radius = int(t.r() * 0.2 + last_Radius * 0.8)
        roi = calculate_roi(current_x, current_y, 80, 80)
        img.draw_rectangle(roi,(0,0,255))
        # 计算与上一次圆心位置和半径的偏差
        deviation = math.sqrt((current_x - last_x) ** 2 + (current_y - last_y) ** 2)
        radius_deviation = abs(Radius - last_Radius)
        # 如果偏差过大，忽略本次更新
        if deviation > max_deviation:
            print(f"圆心或半径变化过大，忽略本次结果。偏差: {deviation}, 半径偏差: {radius_deviation}")
            last_x = current_x
            last_y = current_y
            return  # 跳过当前结果
        if current_x>width:
            current_x=width-1
        if current_y>height:
            current_y=height-1
        print(current_x, current_y)
        color = img.get_pixel(current_x, current_y)
        print(color)

        # 输出圆心位置及颜色信息
        message = f"[y,{current_x/width:.4f},{current_y/height:.4f},{determine_color(color,10)}]"  # 消息内容

        # 画出圆心和圆
        img.draw_circle(current_x, current_y, Radius, color=(255, 0, 0))
        img.draw_cross(current_x, current_y, size=5, color=(255, 0, 0))
        # 更新上一次的圆心位置和半径
        last_x, last_y = current_x, current_y
        last_Radius = Radius
    else:
        g=20
        roi = calculate_roi(last_x, last_y, 80+g, 80+g)
        message = "NONE"
        img.draw_rectangle(roi, color=(0, 255, 0), thickness=1)  # 绘制绿色矩形表示 ROI
    print(message)
    uart.write(message.encode())  # 发送字节串






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
        if max_circle is not None:
            img.draw_circle(max_circle.x(), max_circle.y(), max_circle.r(), color=(255, 0, 0))
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


command = 'c '
i = 0
while True:

    i += 1
    if i > 9:
        i = 0
    print(i)
    uart.write(f"{i}\r\n")
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
                v = 1

        except UnicodeDecodeError:
            print("接收到的字节不是有效的UTF-8编码")
    elif command == 'y':  # 低像素寻找圆
        if s == 1:
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QQVGA)
            sensor.set_auto_exposure(False,exposure_us=60000 )
            sensor.set_auto_gain(False, gain_db=14)
            sensor.set_auto_whitebal(False, rgb_gain_db =  (63.2751, 60.206, 61.5473))
            sensor.skip_frames(10)
            s = 0
        roi= calculate_roi(80, 60, 160, 120)
        get_circles(roi=roi, threshold=3000, r_min=10, r_max=30)
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
        get_circles_2(roi=ROI2, threshold=2400, r_min=30, r_max=50)
    elif command == 'l':
        if v == 1:
            sensor.reset()
            sensor.set_vflip(True)
            sensor.set_hmirror(False)
            sensor.set_transpose(True)
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.HQQVGA)
            sensor.set_auto_whitebal(False)
            sensor.set_auto_gain(False)
            sensor.skip_frames(time=10)
            v = 0
        seek_line()
    elif command == 'g':  # 寻线
        if v == 1:
            sensor.reset()
            sensor.set_vflip(True)
            sensor.set_hmirror(False)
            sensor.set_transpose(True)
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.HQQVGA)
            sensor.set_auto_whitebal(False)
            sensor.set_auto_gain(False)
            sensor.skip_frames(time=10)
            v = 0
        find_lines()
