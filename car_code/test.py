#!/usr/bin/env python
# coding=UTF-8
# huanyuRobot

import rospy
import cv2
import cv_bridge
import numpy
import math
import smach
import smach_ros
import os
import serial
import struct
from pyzbar import pyzbar

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from learning_image_transport.msg import AddTwoInts
import threading


def th():
    rospy.spin()


class Follower:
###################################################################################
##
##                               ���ƻ�е��
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
        self.arm_buf[2] = 5  # ���ݳ���N = ���ƶ���ĸ�����3+5
        self.arm_buf[3] = 6  # ������
        self.arm_buf[4] = int(Num)  # ��������

        self.arm_buf[5] = 1  # ����
        self.arm_buf[6] = 0
        self.send_data_to_arm()

###################################################################################
##
##                               ros����е�۳�ʼ��
##
###################################################################################
        
    def __init__(self):
        ####################################################################
        self.arm_buf = [0x00]*256
        self.stty = rospy.get_param('~stty', '/dev/ttyUSB1')
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
##                               ʶ�����
##
###################################################################################
            
    def recognition(self):
        # cap = cv2.VideoCapture(0)
        lower_black = numpy.array([0,  0,  0])
        upper_black = numpy.array([180, 255, 55])
        # ��ȡÿһ֡
        # _, image = cap.read()
        # ����ͼƬ�ߴ�����߼����ٶ�
        # frame = imutils.resize(frame, width=600)
        smg = rospy.wait_for_message('/camera/image', Image, timeout=30)
        image = self.bridge.imgmsg_to_cv2(smg, desired_encoding='bgr8')
        image = cv2.resize(image, (480, 480))
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_black, upper_black)
        # BEGIN CROP
        h, w, d = image.shape
        # ��Сͼ��߶�
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
        # Ѱ��ͼ������
        shap = None
        if len(cnts) > 0:
            # �ҵ������������
            c = max(cnts, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)  # �õ���С��Ӿ��εģ�����(x,y), (��,��), ��ת�Ƕȣ�
            # ��ȡ��С��Ӿ��ε�4����������(ps: cv2.boxPoints(rect) for OpenCV 3.x)
            box = cv2.boxPoints(rect)
            box = numpy.int0(box)
            # ������
            cv2.drawContours(image, [box], 0, (255, 0, 0), 1)
            # �����������
            area = cv2.contourArea(c)
            print area
            # print(area)
            if area > 5000 and area < 12000:
                ratio = box[3]-box[1]
                if abs(ratio[0]) > abs(ratio[1]):
                    # print("����")
                    shap = 0
                else:
                    # print("����")
                    shap = 1
            elif area > 12000:
                # print('����')
                shap = 2
        # END CROP
        # BEGIN FINDER
        M = cv2.moments(mask)
        # print(M)
        if M['m00'] > 0:
            # ͨ�������ҵ����ĵ� 
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            vtherror = cx - w/2
            return cx, cy, shap


###################################################################################
##
##                       ʶ�����Ͼ��룬����������
##
###################################################################################
    # ������㺯�� 
    def distance_to_camera(self, knownWidth, focalLength, perWidth):
        # ���㲢����Ŀ�����嵽������ľ���
        return (knownWidth * focalLength) / perWidth
    
    # ����Ŀ�������ڲ�ͬ�Ƕ�ʱ������ͷ֮��ľ���
    def Correction_distance(self, x,y,inches):
        # ����ͷ���ĵ�λ��,����frame.shape[0]/2��frame.shape[1]/2�����������ĵ�����,�ü���������240����640�����ĵ�����(320,240)����λ��px��
        X_CENTER = 320
        Y_CENTER = 240
        # ��Ӣ�绻�������
        cm=(inches *30.48/ 12)
        return cm
    
    def color_recognition(self, task_num):
        # ͨ������ͷ�궨��ȡ�����ؽ���
        # �������Ӿ��룬��С��С����
        focalLength = 250
        
        # �����ɫ��ͼ��HSV��ֵ
        lower_red = numpy.array([0,100,0])
        upper_red = numpy.array([6,255,255])
    
        lower_red2 = numpy.array([175,100,0])
        upper_red2 = numpy.array([180,255,255])
    
        # ������ɫ��ͼ��HSV��ֵ
        lower_blue = numpy.array([105,150,0])
        upper_blue = numpy.array([110,255,255])
    
        # ������ɫ��ͼ��HSV��ֵ
        lower_green = numpy.array([60,150,0])
        upper_green = numpy.array([70,255,255])

        # ��ȡÿһ֡
        smg = rospy.wait_for_message('/camera/image', Image, timeout=30)
        frame = self.bridge.imgmsg_to_cv2(smg, desired_encoding='bgr8')
        # ����ͼƬ�ߴ�����߼����ٶ�
        #frame = imutils.resize(frame, width=600)
        #frame = cv2.resize(frame, (0, 0), fx=1, fy=1)
        # ���и�˹ģ��
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        # ת����ɫ�ռ䵽HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # ��ͼƬ���ж�ֵ������
        # �Ժ�ɫ���ж�ֵ������ʱ��hsv��Ҫ��������ֵ
        if task_num==1:
            mask = cv2.inRange(hsv, lower_red, upper_red) | cv2.inRange(hsv, lower_red2, upper_red2)
        #����ɫ����ɫ���ж�ֵ������ʱ��hsvֻ��Ҫһ������ֵ
        elif task_num==2:
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
        else:
            mask = cv2.inRange(hsv, lower_green, upper_green)
    
        # ��ʴ����
        mask = cv2.erode(mask, None, iterations=2)
        # ���Ͳ������ȸ�ʴ���������˳�����
        mask = cv2.dilate(mask, None, iterations=2)
    
        cv2.imshow('mask', mask)
    
        # Ѱ��ͼ������
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        
        # �����������һ��������������²���
        if len(cnts) > 0:
            # �ҵ������������
            c = max(cnts, key=cv2.contourArea)
            # ʹ����С���ԲȦ�������������
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # ���������ľ�
            M = cv2.moments(c)
            # ��������������
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # ֻ�����ߴ��㹻�������
            if radius > 10:
                # ������С���Բ
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # ��������
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                # ���Ŀ������������ͷ�е�λ��
                #print('x:','%.2f'%x,'y:','%.2f'%y,'r:','%.2f'%radius,sep=' ')
                # ���ظ���������С�������겢���أ����Լ����������Ŀ��͸�
                marker=cv2.minAreaRect(c)
                #print('mraker',marker[1][0])
                # ������СֽƬ�ı߳�(��λ:inches)
                KNOWN_WIDTH = 2.7559055
                # �������(��λ:inches)
                inches = follower.distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
                # ��������(��λ:cm)
                cm = follower.Correction_distance(x,y,inches)
                return x, cm
            else:
                return 1, 1
        else:
            return 1, 1
                #frame.shape[0]��ͼ��Ĵ�ֱ�ߴ磨�߶ȣ�,frame.shape[1]��ͼ���ˮƽ�ߴ磨���ȣ�
                #cv.putText(frame, "%.2fcm" % cm,(frame.shape[1] - 200, frame.shape[0] - 20), cv.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)

    def regulate_position(self, task_num):
      #320
      while True:
          x = 1
          cm = 1
          x,cm = follower.color_recognition(int(task_num))
          print(x, cm)
          if x==1 or cm == 1:
             self.twist.linear.x = 0.03
             self.cmd_vel_pub.publish(self.twist)
          elif x < 317:
             self.twist.linear.x = 0.03
             self.cmd_vel_pub.publish(self.twist)
          elif x > 323:
             self.twist.linear.x = -0.03
             self.cmd_vel_pub.publish(self.twist)
          elif x>=317 and x<=323:
             self.twist.linear.x = 0.00
             self.cmd_vel_pub.publish(self.twist)
             break
      
      follower.position(0.0, -cm/100)
      return cm/100
    
    def grap_things(self, target):
        back_y_distance = follower.regulate_position(target)
        # ����ץȡ��Ž�ץȡ���Ϸ��ڶ�Ӧ�Ĳ�
        follower.contrl_arm(target+3)
        rospy.sleep(2.)
        # �˻�ʶ����
        follower.position(0.0, back_y_distance)
        follower.contrl_arm(0)
        # ��ʱ2��
        rospy.sleep(2.)
        follower.contrl_arm(3)
        # ��ʱ2��
        rospy.sleep(2.)



###################################################################################
##
##                          ����С��λ��x,y�ֱ�λ��
##
###################################################################################
    
    def position(self, gx, gy):
        while True:
            var=rospy.wait_for_message('/odom',Odometry,timeout=2)
            x=-var.pose.pose.position.x
            y=var.pose.pose.position.y
            print -var.pose.pose.position.x,var.pose.pose.position.y
            if x >= gx:
                self.twist.linear.x=0.0
                self.cmd_vel_pub.publish(self.twist)
                break
            self.twist.linear.x=-0.3    
            self.cmd_vel_pub.publish(self.twist)
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)

        while True:
            var=rospy.wait_for_message('/odom',Odometry,timeout=2)
            x=-var.pose.pose.position.x
            y=var.pose.pose.position.y
            print -var.pose.pose.position.x,var.pose.pose.position.y
            if y >= gy:
                self.twist.linear.y=0.0
                self.cmd_vel_pub.publish(self.twist)
                break
            self.twist.linear.y=0.3
            self.cmd_vel_pub.publish(self.twist)
        self.cmd_vel_pub.publish(self.twist)


    def regulate_position(self):
        follower.contrl_arm(0)
        '''
        self.twist.angular.z = 0.0 #����ʱ�룬��˳ʱ��
        self.twist.linear.x=0.0
        self.twist.linear.y=0.0
        while True:
            self.cmd_vel_pub.publish(self.twist)
        '''

        while True:
            var = rospy.wait_for_message('/info_line',AddTwoInts,timeout=20)
            var = var.result
            line_type = var[1] #0�� 1���� 2���� 3ʮ��
            offset_x = var[2] 
            offset_y = var[3] 
            black_k = var[4]
            print var
            if offset_x > 5 and line_type == 3:
                self.twist.linear.y=0.03
                self.cmd_vel_pub.publish(self.twist)
            if offset_x < -5 and line_type == 3:
                self.twist.linear.y=-0.03
                self.cmd_vel_pub.publish(self.twist)
            if  offset_x > -5 and offset_x < 5 and line_type ==3:
                self.twist.linear.y=0.0
                self.cmd_vel_pub.publish(self.twist)
                break
        while True:
            var = rospy.wait_for_message('/info_line',AddTwoInts,timeout=20)
            var = var.result
            line_type = var[1] #0�� 1���� 2���� 3ʮ��
            offset_x = var[2] 
            offset_y = var[3] 
            black_k = var[4]
            print var
            if offset_y > 5 and line_type == 3:
                self.twist.linear.x=0.03
                self.cmd_vel_pub.publish(self.twist)
            if offset_y < -5 and line_type == 3:
                self.twist.linear.x=-0.03
                self.cmd_vel_pub.publish(self.twist)
            if  offset_y > -5 and offset_y < 5 and line_type == 3 :
                self.twist.linear.x=0.00
                self.cmd_vel_pub.publish(self.twist)
                break

'''
            if black_k == 1:
                if offset_x > 10:
                    self.twist.linear.x=0.1
                    self.cmd_vel_pub.publish(self.twist)

            if k > 5:
                self.twist.angular.z = 0.1
'''
        




###################################################################################
##
##                               ״̬��
##
###################################################################################

# ����״̬

##############################################
##��ʼ���� 0
##�Ե�ʶ����� 1
##ɨ���ά�� 2
##ʶ���ϲ����� 3
##���ϲ����Ϸ���С���м�� 4
##���ϲ����Ϸ���С����߲� 5
##���ϲ����Ϸ���С���ұ߲� 6
##############################################

'''
#�������ڵ㷢��ʵ��
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
        
        self.flag = int(input('����0��1��'))
        self.flag = 1
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'
'''

#��ʼ״̬

class init_shape(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])        

    def execute(self, userdata):
        rospy.loginfo('Executing state init_shape')
        '''
        follower.contrl_arm(1)
        follower.position(0.3,0.0)
        follower.position(0.3,0.3)
        '''
        self.flag = int(input('����0��1��'))
        self.flag = 1
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'





# �ߵ���ά��ʶ����


class goto_QRcode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state goto_QRcode')
        # ���ƻ�е��

        follower.contrl_arm(1)
        follower.position(0.21,0.0)
        follower.position(0.21,0.6)

#        follower.position(0.6, 0.6)
        self.flag = int(input('����0��1��'))
        self.flag = 1
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'


# ��ȡ��ά��


class read_QRcode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                                   output_keys=['sequence'])
        self.bridge = cv_bridge.CvBridge()
        self.testdate = str(0)

    def execute(self, userdata):
        rospy.loginfo('Executing state read_QRcode')
        while(1):
            smg = rospy.wait_for_message('/camera/image', Image, timeout=30)
            frame = self.bridge.imgmsg_to_cv2(smg,desired_encoding='bgr8')
            test = pyzbar.decode(frame)
            for tests in test:
                self.testdate = tests.data.decode('utf-8')
                print(self.testdate)
                if len(self.testdate) >= 2:
                    break
            if len(self.testdate) >= 2:
                break
            '''
            #print 11111
            for tests in test:
                #print 2222
                testdata = tests.data.decode('utf-8')
                if len(testdata) >= 2:
                    print testdata
                    break
            if len(testdata) >= 2:
               break
            '''
        userdata.sequence = self.testdate 
        self.flag = int(input('����0��1��'))
        self.flag = 1
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'


# �ߵ�ԭ������


class goto_material_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                                   input_keys=['sequence'])

    def execute(self, userdata):
        rospy.loginfo('Executing state goto_material_area')
        target = int(userdata.sequence[0])
        print target
        # С��λ��
        follower.position(0.21, 1.80)
        follower.contrl_arm(2)
        # ����С��λ�� 3Ϊץ��ɫ������ 2Ϊץ��ɫ������ 1Ϊץ��ɫ������
        '''
        for target in range(1, 4):
            follower.grap_things(target)        
        follower.regulate_position(target)
        # ץȡ���Ϸ����м��
        follower.contrl_arm(4)
        # ��ʱ2��
        rospy.sleep(3.)
        '''
        
        self.flag = int(input('����0��1��'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# ץȡ�ϲ�������ɫ��ԭ��


class grab_material_upper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state grab_material_upper')
        # ��ʱ2��
        
        rospy.sleep(2.)

        self.flag = int(input('����0��1��'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'
            

# �ߵ��ּӹ���


class goto_process_area1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state goto_process_area1')
        self.flag = int(input('����0��1��'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'


# ��ԭ�Ϸ��ڴּӹ���


class put_material_down_area1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state put_material_down_area1')
        self.flag = int(input('����0��1��'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# ץȡ�ּӹ������ԭ��


class grab_processed_material(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state grab_processed_material')
        self.flag = int(input('����0��1��'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# �ߵ����Ʒ��


class goto_process_area2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state goto_process_area2')
        self.flag = int(input('����0��1��'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# ��ԭ�Ϸ��ڰ��Ʒ��


class put_material_down_area2(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded1', 'succeeded2', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state put_material_down_area2')
        self.flag = int(input('����0��1��2��'))
        if self.flag == 1:
            return 'succeeded1'
        elif self.flag == 2:
            return 'succeeded2'
        else:
            return 'failed'

# ����ԭ������


class backto_material_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state backto_material_area')
        self.flag = int(input('����0��1��'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# ץȡ�²�������ɫ��ԭ��


class grab_material_lower(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state grab_material_lower')
        self.flag = int(input('����0��1��'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# ���


class compelete(smach.State):
    def __init__(self):
        smach.State.__init__(self)

    def execute(self, userdata):
        rospy.loginfo('Executing state compelete')


def main():
    #rospy.init_node('smach_example_state_machine')
    # ����һ��״̬��
    sm = smach.StateMachine(outcomes=['outcome1', 'outcome2'])
    # ��״̬������
    with sm:
        # ʹ��add��������״̬��״̬����������
        smach.StateMachine.add('init_shap', init_shape(),
                               transitions={'succeeded': 'goto_QRcode',
                                            'failed': 'init_shap'})
        # �ߵ���ά��ʶ����
        smach.StateMachine.add('goto_QRcode', goto_QRcode(),
                               transitions={'succeeded': 'QRcode',
                                            'failed': 'goto_QRcode'})

        # ��ȡ��ά��
        smach.StateMachine.add('QRcode', read_QRcode(),
                               transitions={'succeeded': 'goto_material_area',
                                            'failed': 'QRcode'})
        # �ߵ�ԭ������
        smach.StateMachine.add('goto_material_area', goto_material_area(),
                               transitions={'succeeded': 'grab_material_upper',
                                            'failed': 'goto_material_area'})

        # ץȡ�ϲ�������ɫ��ԭ��
        smach.StateMachine.add('grab_material_upper', grab_material_upper(),
                               transitions={'succeeded': 'goto_process_area1',
                                            'failed': 'grab_material_upper'})

        # �ߵ��ּӹ���
        smach.StateMachine.add('goto_process_area1', goto_process_area1(),
                               transitions={'succeeded': 'put_material_down_area1',
                                            'failed': 'goto_process_area1'})

        # ��ԭ�Ϸ��ڴּӹ���
        smach.StateMachine.add('put_material_down_area1', put_material_down_area1(),
                               transitions={'succeeded': 'grab_processed_material',
                                            'failed': 'put_material_down_area1'})

        # ץȡ�ּӹ������ԭ��
        smach.StateMachine.add('grab_processed_material', grab_processed_material(),
                               transitions={'succeeded': 'goto_process_area2',
                                            'failed': 'grab_processed_material'})

        # �ߵ����Ʒ��
        smach.StateMachine.add('goto_process_area2', goto_process_area2(),
                               transitions={'succeeded': 'put_material_down_area2',
                                            'failed': 'goto_process_area2'})
        # ��ԭ�Ϸ��ڰ��Ʒ��
        smach.StateMachine.add('put_material_down_area2', put_material_down_area2(),
                               transitions={'succeeded1': 'backto_material_area',
                                            'succeeded2': 'compelete',
                                            'failed': 'put_material_down_area2'})
        # ����ԭ������
        smach.StateMachine.add('backto_material_area', backto_material_area(),
                               transitions={'succeeded': 'grab_material_lower',
                                            'failed': 'backto_material_area'})
        # ץȡ�²�������ɫ��ԭ��
        smach.StateMachine.add('grab_material_lower', grab_material_lower(),
                               transitions={'succeeded': 'goto_process_area1',
                                            'failed': 'grab_material_lower'})
        # ���
        smach.StateMachine.add('compelete', compelete())

    # �����������ڲ���������
    sis = smach_ros.IntrospectionServer(
        'my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # ��ʼִ��״̬��
    outcome = sm.execute()

    # �ȴ��˳�
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

