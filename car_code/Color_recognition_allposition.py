# 导入所需模块
from cv2 import cv2 as cv
import numpy
#import imutils

def color_recognition(task_num):
    # 打开摄像头
    cap = cv.VideoCapture(0)
    # 定义红色无图的HSV阈值
    lower_red = numpy.array([0,100,0])
    upper_red = numpy.array([6,255,255])

    lower_red2 = numpy.array([175,100,0])
    upper_red2 = numpy.array([180,255,255])

    # 定义蓝色无图的HSV阈值
    lower_blue = numpy.array([105,150,0])
    upper_blue = numpy.array([110,255,255])

    # 定义绿色无图的HSV阈值
    lower_green = numpy.array([70,150,0])
    upper_green = numpy.array([90,255,255])

    while True:
        _, frame = cap.read()
        blurred = cv.GaussianBlur(frame, (11, 11), 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
        cv.imshow('hsv', hsv)
        mask_red = cv.inRange(hsv, lower_red, upper_red) | cv.inRange(hsv, lower_red2, upper_red2)
        mask_blue = cv.inRange(hsv, lower_blue, upper_blue)
        mask_green = cv.inRange(hsv, lower_green, upper_green)
        # 腐蚀操作
        # 膨胀操作，先腐蚀后膨胀以滤除噪声
        mask_red = cv.erode(mask_red, None, iterations=2)
        mask_red = cv.dilate(mask_red, None, iterations=2)
        mask_green = cv.erode(mask_green, None, iterations=2)
        mask_green = cv.dilate(mask_green, None, iterations=2)
        mask_blue = cv.erode(mask_blue, None, iterations=2)
        mask_blue = cv.dilate(mask_blue, None, iterations=2)
        cv.imshow('mask_red', mask_red)
        cv.imshow('mask_blue', mask_blue)
        cv.imshow('mask_green', mask_green)
        # 寻找图中轮廓
        cnts_red = cv.findContours(mask_red.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        cnts_blue = cv.findContours(mask_blue.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        cnts_green = cv.findContours(mask_green.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        # 如果存在至少一个轮廓则进行如下操作
        if len(cnts_red) > 0 and len(cnts_blue) > 0 and len(cnts_green) > 0:
            # 找到面积最大的轮廓
            c_red = max(cnts_red, key=cv.contourArea)
            c_blue = max(cnts_blue, key=cv.contourArea)
            c_green = max(cnts_green, key=cv.contourArea)
            # 使用最小外接圆圈出面积最大的轮廓
            ((x1, y1), radius1) = cv.minEnclosingCircle(c_red)
            ((x2, y2), radius2) = cv.minEnclosingCircle(c_green)
            ((x3, y3), radius3) = cv.minEnclosingCircle(c_blue)
            # 计算轮廓的矩
            M_red = cv.moments(c_red)
            M_green = cv.moments(c_green)
            M_blue = cv.moments(c_blue)
            # 计算轮廓的重心
            center_red = (int(M_red["m10"] / M_red["m00"]), int(M_red["m01"] / M_red["m00"]))
            center_green = (int(M_green["m10"] / M_green["m00"]), int(M_green["m01"] / M_green["m00"]))
            center_blue = (int(M_blue["m10"] / M_blue["m00"]), int(M_blue["m01"] / M_blue["m00"]))
            # 只处理尺寸足够大的轮廓
            if radius1 > 15 and radius2 > 15 and radius3 > 15:
                # 画出最小外接圆,画出重心
                cv.circle(frame, (int(x1), int(y1)), int(radius1), (0, 0, 255), 2)
                cv.circle(frame, center_red, 5, (0, 0, 255), -1)

                cv.circle(frame, (int(x2), int(y2)), int(radius2), (0, 255, 0), 2)
                cv.circle(frame, center_green, 5, (0, 255, 0), -1)
                
                cv.circle(frame, (int(x3), int(y3)), int(radius3), (255, 0, 0), 2)
                cv.circle(frame, center_blue, 5, (255, 0, 0), -1)
                if x1 > x2 > x3:
                    print(123)
                if x1 > x3 > x2:
                    print(132)
                if x2 > x1 > x3:
                    print(213)
                if x2 > x3 > x1:
                    print(231)
                if x3 > x1 > x2:
                    print(312)
                if x3 > x2 > x1:
                    print(321)
        cv.imshow('frame', frame)
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
        '''
        if flag==1:
            break'''

    cap.release()
    cv.destroyAllWindows()


if __name__=="__main__":
    task_num = 1
    color_recognition(task_num)


