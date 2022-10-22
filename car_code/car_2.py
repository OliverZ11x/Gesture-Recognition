#!/usr/bin/env python
# coding=UTF-8
# huanyuRobot
import smach
import smach_ros
import rospy
import cv_bridge

import cv2
import numpy
import math
import os
import serial
import struct
from pyzbar import pyzbar

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading


def th():
    rospy.spin()


class Follower:
###################################################################################
##
##                               控制机械臂
##
###################################################################################
    def send_data_to_arm(self):
        self.arm_buf[0] = 0x55
        self.arm_buf[1] = 0x55
        l = self.arm_buf[2]+2
        send_data = "200"
        # print(arm_buf)
        for i in range(0, l):
            send_data = send_data+struct.pack('B', self.arm_buf[i])
        # print(send_data)
        # print(len(send_data))
        self.ser.write(send_data)


    def contrl_arm(self, Num):
        self.arm_buf[2] = 5  # 数据长度N = 控制舵机的个数×3+5
        self.arm_buf[3] = 6  # 命令编号
        self.arm_buf[4] = int(Num)  # 动作组编号

        self.arm_buf[5] = 1  # 次数
        self.arm_buf[6] = 0
        self.send_data_to_arm()

###################################################################################
##
##                               ros、机械臂初始化
##
###################################################################################
        
    def __init__(self):
        ####################################################################
        self.arm_buf = [0x00]*256
        self.stty = rospy.get_param('~stty', '/dev/ttyUSB0')
        if os.path.exists(self.stty):
            self.ser = serial.Serial(self.stty, 9600, timeout=0.5)
        else:
            print(self.stty+" not exist!")
        
        #################################################################### 
        self.bridge = cv_bridge.CvBridge()
        #self.image_sub = rospy.Subscriber('/camera', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/image_hsv', Image, queue_size=2)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        '''
        self.twist.linear.x=0.2
        self.cmd_vel_pub.publish(self.twist)
        '''
    def fixed(self,gx,gy):
      while True:      
          var=rospy.wait_for_message('/odom',Odometry,timeout=2)
          self.x=var.pose.pose.position.x*81/85
          self.y=var.pose.pose.position.y*76/85
          print(self.x,self.y)
  
          if abs(self.x-gx)<=0.015:
              self.twist.linear.x=0.0
          if abs(self.x-gx)>0.015 and abs(self.x-gx)<=0.05:           
              self.twist.linear.x=0.05*abs(self.x-gx)/(gx-self.x)
          if abs(self.x-gx)>0.05 and abs(self.x-gx)<=0.3:     
              self.twist.linear.x=0.15*abs(self.x-gx)/(gx-self.x)
          if abs(self.x-gx)>0.3 and abs(self.x-gx)<=1.2:
              self.twist.linear.x=0.3*abs(self.x-gx)/(gx-self.x)
          if abs(self.x-gx)>1.2 and abs(self.x-gx)<=1.8:
              self.twist.linear.x=0.4*abs(self.x-gx)/(gx-self.x)
          if abs(self.x-gx)>1.8 and abs(self.x-gx)<=2.4:
              self.twist.linear.x=0.45*abs(self.x-gx)/(gx-self.x)
          print self.twist.linear.x
          
          if abs(self.y-gy)<=0.015:
              self.twist.linear.y=0.0
          if abs(self.y-gy)>0.015 and abs(self.y-gy)<=0.05 :
              self.twist.linear.y=0.05*abs(self.y-gy)/(gy-self.y)
          if abs(self.y-gy)>0.05 and abs(self.y-gy)<=0.3 :
              self.twist.linear.y=0.15*abs(self.y-gy)/(gy-self.y)
          if abs(self.y-gy)>0.3 and abs(self.y-gy)<=1.2:
              self.twist.linear.y=0.3*abs(self.y-gy)/(gy-self.y)
          if abs(self.y-gy)>1.2 and abs(self.y-gy)<=1.8:
              self.twist.linear.y=0.4*abs(self.y-gy)/(gy-self.y)
          if abs(self.y-gy)>1.8 and abs(self.y-gy)<=2.4:
              self.twist.linear.y=0.45*abs(self.y-gy)/(gy-self.y)
          print self.twist.linear.y
  
          self.cmd_vel_pub.publish(self.twist)
          
          if(abs(self.x-gx)<=0.015 and abs(self.y-gy)<=0.015):
              break    
            
###################################################################################
##
##                               识别黑线
##
###################################################################################
            
    def recognition(self):
        # cap = cv2.VideoCapture(0)
        lower_black = numpy.array([0,  0,  0])
        upper_black = numpy.array([180, 255, 55])
        # 读取每一帧
        # _, image = cap.read()
        # 重设图片尺寸以提高计算速度
        # frame = imutils.resize(frame, width=600)
        smg = rospy.wait_for_message('/camera', Image, timeout=30)
        image = self.bridge.imgmsg_to_cv2(smg, desired_encoding='bgr8')
        image = cv2.resize(image, (480, 480))
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


        mask = cv2.inRange(hsv, lower_black, upper_black)
        
        
        
        # BEGIN CROP
        h, w, d = image.shape
        # 缩小图像高度
        search_top = int(1*h/3)
        search_bot = search_top + 160
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        search_right = int(1*w/3)
        search_left = search_top + 160
        mask[0:h, 0:search_right] = 0
        mask[0:h, search_left:w] = 0

        cnts = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        # 寻找图中轮廓
        shap = None
        if len(cnts) > 0:
            # 找到面积最大的轮廓
            c = max(cnts, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)  # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
            # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
            box = cv2.boxPoints(rect)
            box = numpy.int0(box)
            # 画出来
            cv2.drawContours(image, [box], 0, (255, 0, 0), 1)
            # 计算轮廓面积
            area = cv2.contourArea(c)
            print area
            # print(area)
            if area > 5000 and area < 12000:
                ratio = box[3]-box[1]
                if abs(ratio[0]) > abs(ratio[1]):
                    # print("横线")
                    shap = 0
                else:
                    # print("竖线")
                    shap = 1
            elif area > 12000:
                # print('交点')
                shap = 2
        # END CROP
        # BEGIN FINDER
        M = cv2.moments(mask)
        # print(M)
        if M['m00'] > 0:
            # 通过矩阵找到中心点 
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            vtherror = cx - w/2
            return cx, cy, shap

###################################################################################
##
##                       识别物料距离，并矫正距离
##
###################################################################################
    # 距离计算函数 
    def distance_to_camera(self, knownWidth, focalLength, perWidth):
        # 计算并返回目标物体到摄像机的距离
        return (knownWidth * focalLength) / perWidth
    
    # 矫正目标物体在不同角度时与摄像头之间的距离
    def Correction_distance(self, x,y,inches):
        # 摄像头中心点位置,可用frame.shape[0]/2，frame.shape[1]/2函数计算中心点坐标,该计算机画面宽240，长640，中心点坐标(320,240)（单位：px）
        X_CENTER = 320
        Y_CENTER = 240
        # 将英寸换算成厘米
        cm=(inches *30.48/ 12)
        return cm
    
    def color_recognition(self, task_num):
        # 通过摄像头标定获取的像素焦距
        # 调大增加距离，调小减小距离
        focalLength = 250
        
        # 定义红色无图的HSV阈值
        lower_red = numpy.array([0,100,0])
        upper_red = numpy.array([6,255,255])
    
        lower_red2 = numpy.array([175,100,0])
        upper_red2 = numpy.array([180,255,255])
    
        # 定义蓝色无图的HSV阈值
        lower_blue = numpy.array([105,150,0])
        upper_blue = numpy.array([115,255,255])
    
        # 定义绿色无图的HSV阈值
        lower_green = numpy.array([60,150,0])
        upper_green = numpy.array([75,255,255])

        # 读取每一帧
        smg = rospy.wait_for_message('/camera', Image, timeout=20)
        frame = self.bridge.imgmsg_to_cv2(smg, desired_encoding='bgr8')
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
        elif task_num==3:
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
            if radius > 10:
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
                return x, cm
            else:
                return 1, 1
        else:
            return 1, 1
                #frame.shape[0]：图像的垂直尺寸（高度）,frame.shape[1]：图像的水平尺寸（宽度）
                #cv.putText(frame, "%.2fcm" % cm,(frame.shape[1] - 200, frame.shape[0] - 20), cv.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)

    def regulate_position(self, task_num):
    #320
    while True:
        x = 1
        cm = 1
        x,cm = follower.color_recognition(int(task_num))
        print(x, cm)
        if x==1 or cm == 1:
            self.twist.linear.x = 0.05
            self.cmd_vel_pub.publish(self.twist)
        elif x < 318.0:
            self.twist.linear.x = 0.05
            self.cmd_vel_pub.publish(self.twist)
        elif x > 320.0:
            self.twist.linear.x = -0.05
            self.cmd_vel_pub.publish(self.twist)
        elif x<=322.0 and x>=318.0:
            self.twist.linear.x = 0.00
            self.cmd_vel_pub.publish(self.twist)
            break
    
    follower.position(0.0, -cm/100)
    return cm/100
      
    def grap_things(self, target):
        back_y_distance = follower.regulate_position(target)
        # 根据抓取编号将抓取物料放在对应的槽
        follower.contrl_arm(target+3)
        rospy.sleep(6.)
        # 退回识别区
        follower.position(0.0, back_y_distance)
        follower.contrl_arm(3)
        # 延时2秒
        rospy.sleep(2.)
    
    def regulate_position2(self, task_num):
      #320
      while True:
          x = 1
          cm = 1
          x,cm = follower.color_recognition(int(task_num))
          print(x, cm)
          if x==1 or cm == 1:
             self.twist.linear.y = 0.05
             self.cmd_vel_pub.publish(self.twist)
          elif x < 318.0:
             self.twist.linear.y = -0.05
             self.cmd_vel_pub.publish(self.twist)
          elif x > 320.0:
             self.twist.linear.y = 0.05
             self.cmd_vel_pub.publish(self.twist)
          elif x<=322.0 and x>=318.0:
             self.twist.linear.y = 0.00
             self.cmd_vel_pub.publish(self.twist)
             break
      follower.position(cm/100, 0.0)
      return cm/100
         
    def grap_things2(self, target):
        back_y_distance = follower.regulate_position2(target)
        # 根据抓取编号将抓取物料放在对应的槽
        follower.contrl_arm(target+11)
        rospy.sleep(10.)
        # 退回识别区
        follower.position(back_y_distance, 0.0)
        follower.contrl_arm(11)
        # 延时2秒
        rospy.sleep(5.)
    
    
    def circle_recognition(self, task_num):
        # 定义红色无图的HSV阈值
        lower_red = numpy.array([0,0,0])
        upper_red = numpy.array([6,255,255])
    
        lower_red2 = numpy.array([170,0,0])
        upper_red2 = numpy.array([180,255,255])
    
        # 定义蓝色无图的HSV阈值
        lower_blue = numpy.array([100,60,0])
        upper_blue = numpy.array([130,255,255])
    
        # 定义绿色无图的HSV阈值
        lower_green = numpy.array([60,30,120])
        upper_green = numpy.array([70,255,255])
        # 读取每一帧
        # 读取每一帧
        smg = rospy.wait_for_message('/camera', Image, timeout=30)
        frame = self.bridge.imgmsg_to_cv2(smg, desired_encoding='bgr8')
        # 进行高斯模糊
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        # 转换颜色空间到HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # 对图片进行二值化处理
        #对红色进行二值化处理时，hsv需要两个参数值
        if task_num==1:
            mask = cv2.inRange(hsv, lower_red, upper_red) | cv2.inRange(hsv, lower_red2, upper_red2)
        #对蓝色或绿色进行二值化处理时，hsv只需要一个参数值
        elif task_num==3:
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
                print x
                return x
                # flag=1
            else:
                return 1
        else:
            return 1
            
            
    def put_thins_down(self, task_num):
      #320
      while True:
          x = 1
          x= follower.circle_recognition(int(task_num))
          print(x)
          if x==1:
             self.twist.linear.y = 0.03
             self.cmd_vel_pub.publish(self.twist)
          elif x < 317:
             self.twist.linear.y = -0.03
             self.cmd_vel_pub.publish(self.twist)
          elif x > 323:
             self.twist.linear.y = 0.03
             self.cmd_vel_pub.publish(self.twist)
          elif x>=317 and x<=323 :
             self.twist.linear.y = 0.00
             self.cmd_vel_pub.publish(self.twist)
             break
      i = int(task_num)+6
      print 111111
      follower.contrl_arm(str(i))
      rospy.sleep(10.)
      follower.contrl_arm(10)
      rospy.sleep(5.)
      
      

###################################################################################
##
##                          控制小车位移x,y分别位移
##
###################################################################################
    
    def position(self, gx, gy):
        var=rospy.wait_for_message('/odom',Odometry,timeout=2)
        self.x=var.pose.pose.position.x*81/85
        self.y=var.pose.pose.position.y*76/85
        # y方向走
        follower.fixed(self.x, self.y+gy)
        # 读取当前里程计的x, y 值
        var=rospy.wait_for_message('/odom',Odometry,timeout=2)
        self.x=var.pose.pose.position.x*81/85
        self.y=var.pose.pose.position.y*76/85
        # x方向走
        follower.fixed(self.x+gx , self.y)
        
        

###################################################################################
##
##                               黑线寻路径
##
###################################################################################
    def black_line_path(self):
        lower_black = numpy.array([0,  0,  0])
        upper_black = numpy.array([180, 255, 60])
    
        smg = rospy.wait_for_message('/camera', Image, timeout=30)
        image = self.bridge.imgmsg_to_cv2(smg, desired_encoding='bgr8')
    
        image = cv2.resize(image, (640, 480))
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
        x = cv2.inRange(hsv, lower_black, upper_black)
        x_right = cv2.inRange(hsv, lower_black, upper_black)
        x_left = cv2.inRange(hsv, lower_black, upper_black)
    
        y = cv2.inRange(hsv, lower_black, upper_black)
        y_right = cv2.inRange(hsv, lower_black, upper_black)
        y_left = cv2.inRange(hsv, lower_black, upper_black)
    
        # BEGIN CROP
        h, w, d = image.shape
    
    
        # 缩小图像高度
        search_top = int(1*h/3)  # 160
        search_bot = search_top + 160  # 320
    
    
        y_right[0:h, 0:180] = 0
        y_right[0:h, 420:w] = 0
        y_right[0:search_top, 0:w] = 0   #
        y_right[search_top+80:h, 0:w] = 0   #
    
    
        y[0:h, 0:180] = 0
        y[0:h, 420:w] = 0
        y[0:search_top+80, 0:w] = 0   #
        y[search_top+80*2:h, 0:w] = 0   #
    
        y_left[0:h, 0:180] = 0
        y_left[0:h, 420:w] = 0
        y_left[0:search_top+80*2, 0:w] = 0   #
        y_left[search_top+80*3:h, 0:w] = 0   #
    
    
    
    
        x_left[0:h, 0:180] = 0
        x_left[0:h, 260:w] = 0
        x_left[0:search_top, 0:w] = 0   #
        x_left[search_bot:h, 0:w] = 0   #
        #
        x[0:h, 0:260] = 0
        x[0:h, 340:w] = 0
        x[0:search_top, 0:w] = 0   #
        x[search_bot:h, 0:w] = 0   #
        #
        x_right[0:h, 0:340] = 0
        x_right[0:h, 420:w] = 0
        x_right[0:search_top, 0:w] = 0
        x_right[search_bot:h, 0:w] = 0
        #
    
    
        # 寻找图中轮廓
        x_cnts = cv2.findContours(
            x.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        x_cnts_right = cv2.findContours(
            x_right.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        x_cnts_left = cv2.findContours(
            x_left.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    
        y_cnts = cv2.findContours(
            y.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        y_cnts_right = cv2.findContours(
            y_right.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        y_cnts_left = cv2.findContours(
            y_left.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    
    
        x_area=1
        x_area_right=1
        x_area_left=1
    
        if len(x_cnts) > 0:
            # 找到面积最大的轮廓
            c = max(x_cnts, key=cv2.contourArea)
            # 计算轮廓面积
            x_area = cv2.contourArea(c)
        if len(x_cnts_right) > 0:
            # 找到面积最大的轮廓
            c = max(x_cnts_right, key=cv2.contourArea)
            # 计算轮廓面积
            x_area_right = cv2.contourArea(c)
        if len(x_cnts_left) > 0:
            # 找到面积最大的轮廓
            c = max(x_cnts_left, key=cv2.contourArea)
            # 计算轮廓面积
            x_area_left = cv2.contourArea(c)
    
    
    
        y_area=1
        y_area_right=1
        y_area_left=1
        if len(y_cnts) > 0:
            # 找到面积最大的轮廓
            c = max(y_cnts, key=cv2.contourArea)
            # 计算轮廓面积
            y_area = cv2.contourArea(c)
        if len(y_cnts_right) > 0:
            # 找到面积最大的轮廓
            c = max(y_cnts_right, key=cv2.contourArea)
            # 计算轮廓面积
            y_area_right = cv2.contourArea(c)
        if len(y_cnts_left) > 0:
            # 找到面积最大的轮廓
            c = max(y_cnts_left, key=cv2.contourArea)
            # 计算轮廓面积
            y_area_left = cv2.contourArea(c)
    
        
        #print('x_area= ', x_area, '\tx_area_left= ', x_area_left, '\tx_area_right= ', x_area_right)
        #print('y_area= ', y_area, '\ty_area_left= ', y_area_left, '\ty_area_right= ', y_area_right)
        
        return x_area, x_area_left, x_area_right, y_area, y_area_left, y_area_right
    
    def black_line_init(self):
        self.twist.linear.y = 0.1
        self.cmd_vel_pub.publish(self.twist)
        while True:
          x,x_right,x_left,y,y_right,y_left = follower.black_line_path()
          if x_left >=10000:
            self.twist.linear.y = 0.0
            self.cmd_vel_pub.publish(self.twist)
            print(x,x_left,x_right,y,y_left,y_right)
            break
          


    def suibian(self):
        self.twist.linear.x = 0.06
        self.cmd_vel_pub.publish(self.twist)
        while True:
          x,x_right,x_left,y,y_right,y_left = follower.black_line_path()
          if x_right>x_left:
            self.twist.linear.y = 0.02
            self.cmd_vel_pub.publish(self.twist)
          if x_left>x_right:
            self.twist.linear.y = -0.02
            self.cmd_vel_pub.publish(self.twist)
              
      

        


        



###################################################################################
##
##                               状态机
##
###################################################################################

# 定义状态

##############################################
##初始动作 0
##对地识别黑线 1
##扫描二维码 2
##识别上层物料 3
##将上层物料放在小车中间槽 4
##将上层物料放在小车左边槽 5
##将上层物料放在小车右边槽 6
##############################################

'''
#自启动节点发布实例
class init_shape(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.start_sub = rospy.Subscriber('/start_bn', String, self.start_callback)
        self.start_pub = rospy.Publisher('/action_group', String, queue_size = 1)
    
    def start_callback(self, msg):
        self.start_sub = msg.data
        

    def execute(self, userdata):
        rospy.loginfo('Executing state init_shape')
        
        while True:
            #print self.start_sub
            #var=rospy.wait_for_message('/start_bn',String,timeout=0)
            if self.start_sub == 'start':
               break

        self.start_pub.publish('haodnfhsio')
        

        follower.contrl_arm(3)
        rospy.sleep(3.)

        for target in range(1, 3):
            follower.grap_things(target)        
        
        self.flag = int(input('输入0或1：'))
        self.flag = 1
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'
'''

#初始状态

class init_shape(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

    def execute(self, userdata):
        rospy.loginfo('Executing state init_shape')
        '''
        follower.contrl_arm(11)
        rospy.sleep(5.)
        # 抓取粗加工区
        for target in range(1,4):
            print target
            follower.grap_things2(int(target))
        '''
        '''
        follower.contrl_arm(1)
        rospy.sleep(3.)
        follower.black_line_init()
        follower.suibian()
        '''
        
        '''
        self.twist.linear.y = 0.3
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)
        self.twist.linear.y = 0.0
        self.cmd_vel_pub.publish(self.twist)
        '''
        # 放入库存区
        '''
        follower.contrl_arm(11)
        rospy.sleep(5.)
        sequence=[1,2,3]
        follower.put_things_down_position(sequence)
        '''


        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.cmd_vel_pub.publish(self.twist)
        # 控制机械臂
        follower.contrl_arm(0)
        '''
        #测试库存区试抓取上层目标
        follower.contrl_arm(3)
        rospy.sleep(3.)
        for target in range(1, 4):
            follower.grap_things(target)
        '''
        self.flag = int(input('输入0或1：'))
        self.flag = 1
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'





# 走到二维码识别区


class goto_QRcode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state goto_QRcode')
        # 控制机械臂
        follower.contrl_arm(3)

        # 位移
        follower.fixed(0.0, 0.6)
        follower.fixed(0.6, 0.6)
        
        #self.flag = int(input('输入0或1：'))
        self.flag = 1
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'


# 读取二维码


class read_QRcode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                                   output_keys=['sequence'])
        self.bridge = cv_bridge.CvBridge()
        self.testdate = str(0)

    def execute(self, userdata):
        rospy.loginfo('Executing state read_QRcode')
        while(1):
            smg = rospy.wait_for_message('/camera', Image, timeout=30)
            frame = self.bridge.imgmsg_to_cv2(smg,desired_encoding='bgr8')
            test = pyzbar.decode(frame)
            for tests in test:
                self.testdate = tests.data.decode('utf-8')
                print(self.testdate)
                if len(self.testdate) >= 2:
                    break
            if len(self.testdate) >= 2:
                break
        userdata.sequence = self.testdate
        #self.flag = int(input('输入0或1：'))
        self.flag = 1
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'


# 走到原料区域


class goto_material_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state goto_material_area')
        # 小车位移
        # 位移
        follower.fixed(1.3,0.6)
        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# 抓取上层三种颜色的原料


class grab_material_upper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                                   input_keys=['sequence'])

    def execute(self, userdata):
        rospy.loginfo('Executing state grab_material_upper')
        # 矫正小车位置 3为抓绿色的物料 2为抓蓝色的物料 1为抓红色的物料
        for target in userdata.sequence:
            print target
            follower.grap_things(int(target))
        
        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'
            

# 走到粗加工区


class goto_process_area1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state goto_process_area1')
        follower.contrl_arm(10)
        follower.position(0.3,0.6)
        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'


# 将原料放在粗加工区


class put_material_down_area1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                                   input_keys=['sequence'])

    def execute(self, userdata):
        rospy.loginfo('Executing state put_material_down_area1')
        for i in userdata.sequence:
            follower.put_thins_down(i)

        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# 抓取粗加工区域的原料


class grab_processed_material(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                                   input_keys=['sequence'])

    def execute(self, userdata):
        follower.contrl_arm(11)
        rospy.sleep(5.)
        follower.position(-0.3,0.0)

        for target in userdata.sequence:
            print target
            follower.grap_things2(int(target))
        
        rospy.loginfo('Executing state grab_processed_material')
        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'












# 走到半成品区


class goto_process_area2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state goto_process_area2')
        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# 将原料放在半成品区


class put_material_down_area2(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded1', 'succeeded2', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state put_material_down_area2')
        self.flag = int(input('输入0或1或2：'))
        if self.flag == 1:
            return 'succeeded1'
        elif self.flag == 2:
            return 'succeeded2'
        else:
            return 'failed'

# 返回原料区域


class backto_material_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state backto_material_area')
        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# 抓取下层三种颜色的原料


class grab_material_lower(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state grab_material_lower')
        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# 完成


class compelete(smach.State):
    def __init__(self):
        smach.State.__init__(self)

    def execute(self, userdata):
        rospy.loginfo('Executing state compelete')


def main():
    #rospy.init_node('smach_example_state_machine')
    # 创建一个状态机
    sm = smach.StateMachine(outcomes=['outcome1', 'outcome2'])
    # 打开状态机容器
    with sm:
        # 使用add方法添加状态到状态机容器当中
        smach.StateMachine.add('init_shap', init_shape(),
                               transitions={'succeeded': 'goto_QRcode',
                                            'failed': 'init_shap'})
        # 走到二维码识别区
        smach.StateMachine.add('goto_QRcode', goto_QRcode(),
                               transitions={'succeeded': 'QRcode',
                                            'failed': 'goto_QRcode'})

        # 读取二维码
        smach.StateMachine.add('QRcode', read_QRcode(),
                               transitions={'succeeded': 'goto_material_area',
                                            'failed': 'QRcode'})
        # 走到原料区域
        smach.StateMachine.add('goto_material_area', goto_material_area(),
                               transitions={'succeeded': 'grab_material_upper',
                                            'failed': 'goto_material_area'})

        # 抓取上层三种颜色的原料
        smach.StateMachine.add('grab_material_upper', grab_material_upper(),
                               transitions={'succeeded': 'goto_process_area1',
                                            'failed': 'grab_material_upper'})

        # 走到粗加工区
        smach.StateMachine.add('goto_process_area1', goto_process_area1(),
                               transitions={'succeeded': 'put_material_down_area1',
                                            'failed': 'goto_process_area1'})

        # 将原料放在粗加工区
        smach.StateMachine.add('put_material_down_area1', put_material_down_area1(),
                               transitions={'succeeded': 'grab_processed_material',
                                            'failed': 'put_material_down_area1'})

        # 抓取粗加工区域的原料
        smach.StateMachine.add('grab_processed_material', grab_processed_material(),
                               transitions={'succeeded': 'goto_process_area2',
                                            'failed': 'grab_processed_material'})

        # 走到半成品区
        smach.StateMachine.add('goto_process_area2', goto_process_area2(),
                               transitions={'succeeded': 'put_material_down_area2',
                                            'failed': 'goto_process_area2'})
        # 将原料放在半成品区
        smach.StateMachine.add('put_material_down_area2', put_material_down_area2(),
                               transitions={'succeeded1': 'backto_material_area',
                                            'succeeded2': 'compelete',
                                            'failed': 'put_material_down_area2'})
        # 返回原料区域
        smach.StateMachine.add('backto_material_area', backto_material_area(),
                               transitions={'succeeded': 'grab_material_lower',
                                            'failed': 'backto_material_area'})
        # 抓取下层三种颜色的原料
        smach.StateMachine.add('grab_material_lower', grab_material_lower(),
                               transitions={'succeeded': 'goto_process_area1',
                                            'failed': 'grab_material_lower'})
        # 完成
        smach.StateMachine.add('compelete', compelete())

    # 创建并启动内部监测服务器
    sis = smach_ros.IntrospectionServer(
        'my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # 开始执行状态机
    outcome = sm.execute()

    # 等待退出
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    rospy.init_node('follower')
    follower = Follower()
    main()
'''
    add_thread = threading.Thread(target=th)
    add_thread.start()
'''

