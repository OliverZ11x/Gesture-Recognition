from cv2 import cv2
import numpy as np
from matplotlib import pyplot as plt
import imutils

# 打开摄像头
cap = cv2.VideoCapture(0)

while True:
    # 读取每一帧
    _, frame = cap.read()
    # 重设图片尺寸以提高计算速度
    frame = imutils.resize(frame, width=1000)
        
    #image=cv2.imread('red3.jpg')
    HSV=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    def getpos(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN: #定义一个鼠标左键按下去的事件
            print(HSV[y,x],sep=",")
    cv2.imshow("imageHSV",HSV)
    cv2.setMouseCallback("imageHSV",getpos)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

#cv2.imshow('image',image)

#cv2.waitKey(0)

cap.release()
cv2.destroyAllWindows()

