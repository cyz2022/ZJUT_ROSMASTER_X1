import cv2 as cv
import time
cap = cv.VideoCapture(0,cv.CAP_V4L2)
cap.set(cv.CAP_PROP_FPS, 120)
cap.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
time1=0
cnt=0
while True:
    start_time=time.time()
    cnt+=1
    ret, frame = cap.read()
    end_time = time.time()
    #print(center,flag)
    time1=time1+end_time-start_time
    cv.imshow('frame',frame)
    cv.imwrite('111.png',frame)
    print(1/(end_time-start_time))
    if cv.waitKey(1) & 0xFF == ord(' '):
        break
cap.release()
cv.destroyAllWindows()
