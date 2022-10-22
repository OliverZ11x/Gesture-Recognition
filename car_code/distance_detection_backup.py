import math
from cv2 import cv2
import numpy

###################################################################################
##
##                       识别物料距离，并矫正距离
##
###################################################################################

class Follower():
    # 距离计算函数
    def distance_to_camera(self, knownWidth, focalLength, perWidth):
        # 计算并返回目标物体到摄像机的距离
        return (knownWidth * focalLength) / perWidth

    # 矫正目标物体在不同角度时与摄像头之间的距离
    def Correction_distance(self, x,y,inches):
        # 摄像头中心点位置,可用frame.shape[0]/2，frame.shape[1]/2函数计算中心点坐标,该计算机画面宽240，长640，中心点坐标(320,240)（单位：px）
        X_CENTER = 320
        Y_CENTER = 240
        cm=(inches *30.48/ 12)
        return cm

    def color_recognition(self, task_num):
        # 通过摄像头标定获取的像素焦距
        focalLength = 390
        
        # 定义红色无图的HSV阈值
        lower_red = numpy.array([0,100,0])
        upper_red = numpy.array([6,255,255])

        lower_red2 = numpy.array([175,100,0])
        upper_red2 = numpy.array([180,255,255])

        # 定义蓝色无图的HSV阈值
        lower_blue = numpy.array([105,150,0])
        upper_blue = numpy.array([110,255,255])

        # 定义绿色无图的HSV阈值
        lower_green = numpy.array([60,150,0])
        upper_green = numpy.array([70,255,255])

        # 读取每一帧
        cap = cv2.VideoCapture(0)
        while True:
            _, frame = cap.read()
            # 重设图片尺寸以提高计算速度
            #frame = imutils.resize(frame, width=600)
            #frame = cv2.resize(frame, (0, 0), fx=1, fy=1)
            # 进行高斯模糊
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            # 转换颜色空间到HSV
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            # 对图片进行二值化处理
            # 对红色进行二值化处理时，hsv需要两个参数值
            if task_num==1:
                mask = cv2.inRange(hsv, lower_red, upper_red) | cv2.inRange(hsv, lower_red2, upper_red2)
            #对蓝色或绿色进行二值化处理时，hsv只需要一个参数值
            elif task_num==2:
                mask = cv2.inRange(hsv, lower_blue, upper_blue)
            else:
                mask = cv2.inRange(hsv, lower_green, upper_green)

            # 腐蚀操作
            mask = cv2.erode(mask, None, iterations=2)
            # 膨胀操作，先腐蚀后膨胀以滤除噪声
            mask = cv2.dilate(mask, None, iterations=2)

            cv2.imshow('mask', mask)

            # 寻找图中轮廓
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            
            
            # 如果存在至少一个轮廓则进行如下操作
            if len(cnts) > 0:
                # 找到面积最大的轮廓
                c = max(cnts, key=cv2.contourArea)
                # 使用最小外接圆圈出面积最大的轮廓
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                # 计算轮廓的矩
                M = cv2.moments(c)
                # 计算轮廓的重心
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # 只处理尺寸足够大的轮廓
                if radius > 15:
                    # 画出最小外接圆
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # 画出重心
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    # 输出目标物体在摄像头中的位置
                    #print('x:','%.2f'%x,'y:','%.2f'%y,'r:','%.2f'%radius,sep=' ')
                    # 返回该轮廓的最小矩形坐标并返回，用以计算测量物体的宽和高
                    marker=cv2.minAreaRect(c)
                    #print('mraker',marker[1][0])
                    # 正方形小纸片的边长(单位:inches)
                    KNOWN_WIDTH = 2.7559055
                    # 计算距离(单位:inches)
                    inches = follower.distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
                    # 矫正距离(单位:cm)
                    cm = follower.Correction_distance(x,y,inches)
                    x,cm = follower.color_recognition(int(task_num))
                    print(x, cm, -cm/100+0.05)
                    # frame.shape[0]：图像的垂直尺寸（高度）,frame.shape[1]：图像的水平尺寸（宽度）
                    cv2.putText(frame, "%.2fcm" % cm,(frame.shape[1] - 200, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)
                    cv2.imshow('frame',frame)

                    
            else:
                return 1, 1

    def regulate_position(self, task_num):
        #320
        while True:
            x,cm = follower.color_recognition(int(task_num))
            print(x, cm, -cm/100+0.05)

follower = Follower()
follower.color_recognition(3)