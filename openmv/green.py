from pyb import LED
import sensor
import image
import time
import pyb
from machine import UART
uart = UART(3, baudrate=115200)
red_led = LED(1)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_exposure(False, exposure_us=60000)
sensor.set_auto_gain(False, gain_db=14)
sensor.set_auto_whitebal(False, rgb_gain_db=(63.2751, 60.206, 61.5473))
sensor.skip_frames(20)
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


i = 0
while True:
    i += 1
    if i > 9:
        i = 0
    # print(i)
    # s2=456
    # s1=321
    # msg3=f"[e,{s2},{s1}]"
    # uart.write(msg3+'\r\n')
    # uart.write(f"{i}\r\n")
    get_qrcode()
