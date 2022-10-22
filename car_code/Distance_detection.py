# 导入所需模块
from cv2 import cv2 as cv
import numpy as np
import math

###################################################################################
##
##                       识别物料距离
##
###################################################################################
# 正方形小纸片的边长(单位:inches)
KNOWN_WIDTH = 2.7559055
# 距离计算函数 
def distance_to_camera(knownWidth, focalLength, perWidth):
    # 计算并返回目标物体到摄像机的距离
    return (knownWidth * focalLength) / perWidth

# 摄像头中心点位置,可用frame.shape[0]/2，frame.shape[1]/2函数计算中心点坐标,该计算机画面宽240，长640，中心点坐标(320,240)（单位：px）
X_CENTER = 320
Y_CENTER = 240
# 矫正目标物体在不同角度时与摄像头之间的距离
def Correction_distance(x,y,inches):

    # 计算目标物体坐标与中心点坐标之间的距离
    distance=math.sqrt(math.pow(Y_CENTER-y,2)+math.pow(X_CENTER-x,2))
    #print('x,y:','%.2f'%x,'%.2f'%y,'X,Y:',X_CENTER,Y_CENTER,'distance:','%.2f'%distance)
    #print('inches:','%.2f'%inches,'distance:','%.2f'%distance)
    # 计算矫正实际距离
    # tan=distance/inches
    # 将英寸换算成厘米
    cm=(inches *30.48/ 12)
    return cm

# 22  23  25


def color_recognition(task_num):
    # 通过摄像头标定获取的像素焦距
    #29.5
    focalLength = 390
    # 打开摄像头
    cap = cv.VideoCapture(0)
    # 定义红色无图的HSV阈值
    lower_red = np.array([0,100,0])
    upper_red = np.array([6,255,255])

    lower_red2 = np.array([175,100,0])
    upper_red2 = np.array([180,255,255])

    # 定义蓝色无图的HSV阈值
    lower_blue = np.array([105,150,0])
    upper_blue = np.array([115,255,255])

    # 定义绿色无图的HSV阈值
    lower_green = np.array([60,150,0])
    upper_green = np.array([75,255,255])

    while True:
        # 读取每一帧
        _, frame = cap.read()
        # 重设图片尺寸以提高计算速度
        #frame = imutils.resize(frame, width=600)
        #frame = cv.resize(frame, (0, 0), fx=1, fy=1)
        # 进行高斯模糊        
        blurred = cv.GaussianBlur(frame, (11, 11), 0)
        # 转换颜色空间到HSV
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

        #cv.imshow('hsv', hsv)

        # 对图片进行二值化处理
        # 对红色进行二值化处理时，hsv需要两个参数值
        if task_num==1:
            mask = cv.inRange(hsv, lower_red, upper_red) | cv.inRange(hsv, lower_red2, upper_red2)
        #对蓝色或绿色进行二值化处理时，hsv只需要一个参数值
        elif task_num==2:
            mask = cv.inRange(hsv, lower_blue, upper_blue)
        else:
            mask = cv.inRange(hsv, lower_green, upper_green)

        # 腐蚀操作
        mask = cv.erode(mask, None, iterations=2)
        # 膨胀操作，先腐蚀后膨胀以滤除噪声
        mask = cv.dilate(mask, None, iterations=2)
    
        cv.imshow('mask', mask)
    
        # 寻找图中轮廓
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        

        # 如果存在至少一个轮廓则进行如下操作
        if len(cnts) > 0:
            # 找到面积最大的轮廓
            c = max(cnts, key=cv.contourArea)
            # 使用最小外接圆圈出面积最大的轮廓
            ((x, y), radius) = cv.minEnclosingCircle(c)
            # 计算轮廓的矩
            M = cv.moments(c)
            # 计算轮廓的重心
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # 只处理尺寸足够大的轮廓
            if radius > 15:
                # 画出最小外接圆
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # 画出重心
                cv.circle(frame, center, 5, (0, 0, 255), -1)
                # 输出目标物体在摄像头中的位置
                #print('x:','%.2f'%x,'y:','%.2f'%y,'r:','%.2f'%radius,sep=' ')
                # 返回该轮廓的最小矩形坐标并返回，用以计算测量物体的宽和高
                marker=cv.minAreaRect(c)
                #print('mraker',marker[1][0])
                # 计算距离(单位:inches)
                inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
                # 矫正距离(单位:cm)
                cm=Correction_distance(x,y,inches)
                print(radius, cm)
                #frame.shape[0]：图像的垂直尺寸（高度）,frame.shape[1]：图像的水平尺寸（宽度）
                cv.putText(frame, "%.2fcm" % cm,(frame.shape[1] - 200, frame.shape[0] - 20), cv.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)
                

        cv.imshow('frame', frame)
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break

    cap.release()
    cv.destroyAllWindows()


if __name__=="__main__":
    #task_num=int(input('输入任务号：'))
    task_num=1
    color_recognition(task_num)

###################################################################################



