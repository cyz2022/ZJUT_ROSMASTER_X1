import pygame
import time
import numpy as np
import cv2 as cv
from Rosmaster_Lib import Rosmaster
import pyzbar.pyzbar as pyzbar

bot = Rosmaster()

def voice_behavior(choice):
    try:
        pygame.mixer.init()
        pygame.mixer.music.set_volume(1.0)
        choice = str(choice)
        if choice.isdigit():
            number_or_file = int(choice)
            if number_or_file < 2 or number_or_file > 8:
                print("输入的数字必须在2到8之间。")
                return
            file_path = "./voice/{}.mp3".format(number_or_file)
        elif isinstance(choice, str):
            if choice == 's':
                file_path = './voice/scenic.mp3'
            elif choice == 't':
                file_path = './voice/treasure.mp3'
            elif choice == 'p':
                file_path = './voice/prepared.mp3'
            else:
                print("无效的指令或文件名。")
                return
        else:
            print("无效的参数类型。")
            return

        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            continue
    except KeyboardInterrupt:
        pygame.mixer.music.stop()


#def voice_behavior(number):
#    if number < 2 or number > 8:
#        print("输入的数字必须在2到8之间。")
##        return
#
##    mp3_file_path = f"{//voice//number}.mp3"
 #   try:
 #       pygame.mixer.init()
 #       pygame.mixer.music.set_volume(1.0)
 #       pygame.mixer.music.load(mp3_file_path)
 #       pygame.mixer.music.play()
 #       while pygame.mixer.music.get_busy():
 #           continue
 #   except KeyboardInterrupt:
 #       pygame.mixer.music.stop()

def servo_behavior():
    bot.set_pwm_servo(3,0)
    time.sleep(0.5)
    bot.set_pwm_servo(3,150)


def decodeIdentify(image):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    barcodeData = ''
    barcodes = pyzbar.decode(gray)
    for barcode in barcodes:
        encoding = 'UTF-8'
        barcodeData = barcode.data.decode(encoding)
    return barcodeData

def get_barcodeData():
    capture = cv.VideoCapture('/dev/camera3')
    QRcode = ""
    print(capture.isOpened())
    while capture.isOpened() == False:
        continue
    while capture.isOpened():
        ret, frame = capture.read()
        #cv.imshow("frame",frame)
        #action = cv.waitKey(10) & 0xFF
        QRcode = decodeIdentify(frame)
        if QRcode != "":
            capture.release()
            return QRcode
    
    

def color_recognition():

    #print("进入颜色识别")
    red = 0
    yellow = 0
    green = 0
    #red_readin = 0
    #yellow_readin = 0
    #green_readin=0
    # 颜色范围（请根据需要修改）
    lower_yellow = np.array([26,43,46])
    upper_yellow = np.array([39,255,255])

    lower_green = np.array([40,43,46])
    upper_green = np.array([90,255,255])

    lower_red1 = np.array([0,43,46])
    upper_red1 = np.array([10,255,255])

    lower_red2 = np.array([156,43,46])
    upper_red2 = np.array([180,255,255])

    # 创建颜色名称列表
    color_names = ['Yellow', 'Green', 'Red']

    # 颜色纯色值
    yellow_color = (0, 255, 255)  # 黄色
    green_color = (0, 255, 0)  # 绿色
    red_color = (0, 0, 255)  # 红色

    # 捕获摄像头视频
    capture = cv.VideoCapture('/dev/camera2')
    print("open",capture.isOpened())
    while capture.isOpened() == False:
        continue
    while capture.isOpened():
        ret, frame = capture.read()

        height, width = frame.shape[:2]
        startRow, startCol = int(height * .4), int(width * .4)
        endRow, endCol = int(height * .6), int(width * .6)
        croppedImage = frame[startRow:endRow, startCol:endCol]
        hsv_frame = cv.cvtColor(croppedImage, cv.COLOR_BGR2HSV)

        yellow_mask = cv.inRange(hsv_frame, lower_yellow, upper_yellow)
        green_mask = cv.inRange(hsv_frame, lower_green, upper_green)
        red_mask1 = cv.inRange(hsv_frame, lower_red1, upper_red1)
        red_mask2 = cv.inRange(hsv_frame, lower_red2, upper_red2)
        red_mask = cv.bitwise_or(red_mask1, red_mask2)
        
        # 计算各颜色区域的像素数量
        yellow_pixels = cv.countNonZero(yellow_mask)
        green_pixels = cv.countNonZero(green_mask)
        red_pixels = cv.countNonZero(cv.bitwise_or(red_mask1, red_mask2))

        if yellow_pixels == 0 and green_pixels == 0 and red_pixels == 0:
            print("No Detection")
            red=0
            yellow=0
            green=0
        else:
            # 将颜色像素数量存入列表
            pixel_counts = [yellow_pixels, green_pixels, red_pixels]

            # 找到出现最多的颜色的索引
            max_count_index = np.argmax(pixel_counts)
            if max_count_index == 0:
                yellow += 1
            if max_count_index == 1:
                green += 1
            if max_count_index == 2:
                red += 1
            if yellow>5:   
                capture.release()
                print("yellow")
                return 0
            
            if green >5:
                capture.release()
                print("green")
                return 1

            if red >5:
                capture.release()
                print("red")
                return 2

    


if __name__ == "__main__":
   # while 1:
       # color_recognition()
    #bot.set_motor(30,30,30,30)
    #voice_behavior("p")
    #get_barcodeData()
    while 1:
        print(color_recognition())
    #barcodeData = get_barcodeData()
    #while barcodeData == "":
    #    barcodeData = get_barcodeData()
    #print(barcodeData)

