import sensor, image, time

# 初始化传感器
sensor.reset()
sensor.set_vflip(True)  # 垂直翻转
sensor.set_hmirror(True)  # 水平翻转
sensor.set_pixformat(sensor.RGB565)  # 使用 RGB565 格式
sensor.set_framesize(sensor.HQVGA)  # 设置分辨率为 QQVGA (160x120)
sensor.set_auto_exposure(False, exposure_us=30000)
sensor.set_auto_gain(False, gain_db=12)
sensor.set_auto_whitebal(False, rgb_gain_db=(62.7165, 60.2071, 61.911))
sensor.skip_frames(20)  # 等待相机稳定

clock = time.clock()
def calculate_roi(center_x, center_y, width, height):
    # 计算矩形的左上角坐标
    x = int(center_x - width / 2)
    y = int(center_y - height / 2)

    # 返回 ROI，格式为 (x, y, w, h)
    return (x, y, width, height)
# 定义多个颜色的阈值范围
thresholds = [
    (55, 66, 15, 26, -4, 8),  # 红色
    (59, 70, -9, -1, -7, 8),  # 绿色
    (54, 71, 2, 7, -14, -8),  # 蓝色
]
while True:
    target_blob = None
    target_dis = 999999
    clock.tick()
    img = sensor.snapshot()  # 获取当前图像
    width = sensor.width()
    height = sensor.height()
    ROI =calculate_roi((width / 2),(height / 2),int(width*0.8),int(height*0.8))

    # 使用阈值列表，查找多个颜色的色块
    blobs = img.find_blobs(thresholds, area_threshold=10, merge=True)  # 查找符合阈值范围的色块
    if blobs:
        for blob in blobs:
            if blob.density() > 0.2:
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
            print(target_color)
            img.binary([thresholds[target_color - 1]])  # 阈值范围
            # img.erode(1)  # 腐蚀
            #img.dilate(2)  # 膨胀
            #img.gaussian(1)#高斯滤波
            #img.mean(2)#均值滤波
            #img.median(1, percentile=0.5)#中值滤波
            img.midpoint(1, bias=0.5)#中点滤波
            threshold = [30]  # 单一阈值作为列表传递
            img.binary([threshold])
            img.mean(2)#均值滤波

            #img.dilate(2)  # 膨胀

            # 查找圆形，调整下面的参数以适应你的场景
            circles = img.find_circles(roi=ROI,threshold=2500,  r_min=30, r_max=40, r_step=2)
            # 绘制找到的圆
            if circles:
                for circle in circles:
                    img.draw_circle(circle.x(), circle.y(), circle.r(), color=(255, 0, 0))  # 绘制红色圆
                    img.draw_cross(circle.x(), circle.y(), color=(0, 255, 0))  # 绘制绿色交叉线，圆心位置

    # 显示帧率
    # print("FPS:", clock.fps())
