import cv2 as cv
import time
import numpy as np
from Rosmaster_Lib import Rosmaster

def get_dir(img):
    cv.imshow("img",img)
    # 应用高斯模糊以减少噪声
    img = cv.GaussianBlur(img, (5, 5), 0)

    # 创建 img 的副本 img1
    img1 = img.copy()

    # 将上半部分设置为黑色
    img1[:280, :] = 0

    # 截取下半部分
    img_lower = img[280:640, :]

    # 转换为灰度图
    gray_lower = cv.cvtColor(img_lower, cv.COLOR_BGR2GRAY)
    #cv.imshow("Gray Image", gray_lower)

    # 使用大津法进行二值化

    ret, binary_lower = cv.threshold(gray_lower, 30, 255, cv.THRESH_BINARY)
    #cv.imshow("111",binary_lower)
    binary_lower = cv.bitwise_not(binary_lower)

    # 创建核进行膨胀和腐蚀处理
    kernel = np.ones((3,3), np.uint8)

    # 腐蚀处理
    eroded = cv.erode(binary_lower, kernel, iterations=1)

    # 提取轮廓
    contours, hierarchy = cv.findContours(eroded, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if(len(contours)==0):
        return 320,0
    # 找到最大的轮廓
    max_contour = max(contours, key=cv.contourArea)

    # 创建一个与 img_lower 相同大小的黑色图像
    mask = np.zeros_like(eroded)

    # 绘制最大的轮廓，将其填充为白色
    cv.drawContours(mask, [max_contour], -1, 255, thickness=cv.FILLED)

    # 将掩码应用于原始图像的下半部分
    white_fill = np.full_like(img_lower, 255)
    img_lower_masked = cv.bitwise_and(white_fill, white_fill, mask=mask)

    # 将处理后的结果放回原图像副本的相应位置
    img1[280:640, :] = img_lower_masked

    # 腐蚀处理
    img1 = cv.erode(img1, kernel, iterations=2)

    sorted_contour = sorted(max_contour, key=lambda point: point[0][1])

    points_by_y = {}
    for point in sorted_contour:
        x, y = point[0]
        if y not in points_by_y:
            points_by_y[y] = []
        points_by_y[y].append(x)

    direction = []
    for y, x_values in points_by_y.items():
        x_values.sort()
        for i in range(0, len(x_values), 3):
            segment = x_values[i:i+5]
            avg_x = int(np.mean(segment))
            cv.circle(img1, (avg_x, y+280), 3, (0, 0, 255), -1)
            direction.append(avg_x)

    direction_out = np.mean(direction)
    return direction_out, img1

def control_bot(center_x, width, bot, prev_time):
    # 根据黑线的中心位置调整小车运动方向
    error = center_x - width // 2
    
    error = error/60

    if error < 0:
        error -= 0.15
    else:
        error += 0.15
    
    if abs(error)<0.2:
        error = 0
    
    if error>0:
        bot.set_car_motion(0,0,-0.5)
    elif error<0:
        bot.set_car_motion(0,0,0.5)
    if error == 0:
        bot.set_car_motion(0,0,0)
    
    return 0

if __name__ == '__main__':
    bot = Rosmaster()
    prev_time = time.time()

    cap = cv.VideoCapture(1)
    time.sleep(3)
    cap.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    
    count = 0
    while True:
        ret, frame = cap.read()
        count +=1
        if count<3:
            continue 
        if not ret:
            print("no cap")
            break

        center_x, binary_frame = get_dir(frame)
        prev_time = control_bot(center_x, frame.shape[1], bot, prev_time)

        #cv.imshow('frame', frame)
        cv.imshow('binary', binary_frame)
       
        if cv.waitKey(1) & 0xFF == ord(' '):
            break

    cap.release()
    cv.destroyAllWindows()
    bot.set_car_motion(0, 0, 0)  # 停止小车
