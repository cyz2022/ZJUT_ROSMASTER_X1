import cv2 as cv

video = cv.VideoCapture()
frame,ret = video.read()
while 1:
    cv.imshow('frame',frame)

