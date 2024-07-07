import cv2
import numpy as np

def get_dir(image_path):

    # 读取图像
    img = cv2.imread(image_path)

    # 应用高斯模糊以减少噪声
    img = cv2.GaussianBlur(img, (5, 5), 0)

    # 创建 img 的副本 img1
    img1 = img.copy()

    # 将上半部分设置为黑色
    img1[:280, :] = 0

    # 截取下半部分
    img_lower = img[280:640, :]

    # 转换为灰度图
    gray_lower = cv2.cvtColor(img_lower, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Gray Image", gray_lower)

    # 使用大津法进行二值化
    ret, binary_lower = cv2.threshold(gray_lower, 45, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    binary_lower = cv2.bitwise_not(binary_lower)

    # 创建核进行膨胀和腐蚀处理
    kernel = np.ones((3,3 ), np.uint8)


    # 腐蚀处理
    eroded = cv2.erode(binary_lower, kernel, iterations=1)

    # 提取轮廓
    contours, hierarchy = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 找到最大的轮廓
    max_contour = max(contours, key=cv2.contourArea)


    # 创建一个与 img_lower 相同大小的黑色图像
    mask = np.zeros_like(eroded)

    # 绘制最大的轮廓，将其填充为白色
    cv2.drawContours(mask, [max_contour], -1, 255, thickness=cv2.FILLED)

    # 将掩码应用于原始图像的下半部分
    white_fill = np.full_like(img_lower, 255)
    img_lower_masked = cv2.bitwise_and(white_fill, white_fill, mask=mask)

    # 将处理后的结果放回原图像副本的相应位置
    img1[280:640, :] = img_lower_masked

    # 腐蚀处理
    img1 = cv2.erode(img1, kernel, iterations=2)

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
            cv2.circle(img1, (avg_x, y+280), 3, (0, 0, 255), -1)
            direction.append(avg_x)

    direction_out = np.mean(direction)
    return direction_out, img1
    # print(direction_out)
    # cv2.circle(img1, (int(direction_out), 280), 3, (0, 0, 255), -1)

    # # 显示灰度图、二值化结果、膨胀结果、腐蚀结果、最大轮廓掩码、掩码应用结果和修改后的原图像
    # cv2.imshow('Masked Image', img_lower_masked)
    # cv2.imshow('Modified Original Image', img1)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

