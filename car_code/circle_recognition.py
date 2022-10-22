# 导入所需模块
from cv2 import cv2
import numpy as np
#import imutils

def color_recognition(task_num):
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    # 定义红色无图的HSV阈值
    lower_red = np.array([0,0,0])
    upper_red = np.array([6,255,255])

    lower_red2 = np.array([170,0,0])
    upper_red2 = np.array([180,255,255])

    # 定义蓝色无图的HSV阈值
    lower_blue = np.array([100,60,0])
    upper_blue = np.array([130,255,255])

    # 定义绿色无图的HSV阈值
    lower_green = np.array([60,30,120])
    upper_green = np.array([75,255,255])


    while True:
        # 读取每一帧
        _, frame = cap.read()
        # 重设图片尺寸以提高计算速度
        #frame = imutils.resize(frame, width=600)
        #frame = cv2.resize(frame, (0, 0), fx=1, fy=1)
        # 进行高斯模糊
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        # 转换颜色空间到HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # 对图片进行二值化处理
        #对红色进行二值化处理时，hsv需要两个参数值
        if task_num==1:
            mask = cv2.inRange(hsv, lower_red, upper_red) | cv2.inRange(hsv, lower_red2, upper_red2)
        #对蓝色或绿色进行二值化处理时，hsv只需要一个参数值
        elif task_num==2:
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
        else:
            #mask = cv2.inRange(hsv, lower_green, upper_green)
            mask = cv2.inRange(hsv, lower_green, upper_green)

        # 腐蚀操作
        mask = cv2.erode(mask, None, iterations=0)
        # 膨胀操作，先腐蚀后膨胀以滤除噪声
        mask = cv2.dilate(mask, None, iterations=15)
        # 寻找图中轮廓
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        # 如果存在至少一个轮廓则进行如下操作
        if len(cnts) > 0:
            # 找到面积最小的轮廓
            c = max(cnts, key=cv2.contourArea)
            # 使用最小外接圆圈出面积最大的轮廓
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # 计算轮廓的矩
            M = cv2.moments(c)
            # 计算轮廓的重心
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # 只处理尺寸足够大的轮廓
            if radius > 30:
                # 画出最小外接圆
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # 画出重心
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                # 输出目标物体在摄像头中的位置
                print('%.2f'%x,'%.2f'%y,'%.2f'%radius,sep='\t')
                # flag=1

        cv2.imshow('mask', mask)
        cv2.imshow('frame', frame)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
        
    cap.release()
    cv2.destroyAllWindows()


if __name__=="__main__":
    #task_num=int(input('输入任务号：'))
    task_num = 1
    color_recognition(task_num)
