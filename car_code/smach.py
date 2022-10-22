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

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading


def th():
    rospy.spin()


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            '/camera', Image, self.image_callback)
        # self.sub_odom  = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.image_pub = rospy.Publisher('/image_hsv', Image, queue_size=2)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

        self.forward_velocity = rospy.get_param('~forward_velocity', 0.1)
        self.scale_diversion = rospy.get_param('~scale_diversion', 0.4)
        self.a = 0.0
        self.b = 0.0
    '''def odom_callback(self, msg):
    self.x=msg.pose.pose.position.x
    self.y=msg.pose.pose.position.y 
    self.z=msg.pose.pose.position.z
    print self.x,self.y,self.z
    self.a = 1'''

    def fixed(self, gx, gy):
        self.twist.angular.z = 0.0
        while True:
            var = rospy.wait_for_message('/odom', Odometry, timeout=2)
            self.x = var.pose.pose.position.x*79/85
            self.y = var.pose.pose.position.y*79/85
            print(self.x, self.y)
            if abs(self.x-gx) <= 0.02:
                self.twist.linear.x = 0.0
            if abs(self.x-gx) > 0.02 and abs(self.x-gx) <= 0.05:
                self.twist.linear.x = 0.01*abs(self.x-gx)/(gx-self.x)
            if abs(self.x-gx) > 0.05 and abs(self.x-gx) <= 0.15:
                self.twist.linear.x = 0.1*abs(self.x-gx)/(gx-self.x)
            if abs(self.x-gx) > 0.15 and abs(self.x-gx) <= 0.6:
                self.twist.linear.x = 0.15*abs(self.x-gx)/(gx-self.x)
            if abs(self.x-gx) > 0.6 and abs(self.x-gx) <= 1.2:
                self.twist.linear.x = 0.3*abs(self.x-gx)/(gx-self.x)
            if abs(self.x-gx) > 1.2 and abs(self.x-gx) <= 1.8:
                self.twist.linear.x = 0.4*abs(self.x-gx)/(gx-self.x)
            if abs(self.x-gx) > 1.8 and abs(self.x-gx) <= 2.4:
                self.twist.linear.x = 0.5*abs(self.x-gx)/(gx-self.x)

            if abs(self.y-gy) <= 0.02:
                self.twist.linear.y = 0.0
            if abs(self.y-gy) > 0.02 and abs(self.y-gy) <= 0.05:
                self.twist.linear.y = 0.01*abs(self.y-gy)/(gy-self.y)
            if abs(self.y-gy) > 0.05 and abs(self.y-gy) <= 0.15:
                self.twist.linear.y = 0.1*abs(self.y-gy)/(gy-self.y)
            if abs(self.y-gy) > 0.15 and abs(self.y-gy) <= 0.6:
                self.twist.linear.y = 0.15*abs(self.y-gy)/(gy-self.y)
            if abs(self.y-gy) > 0.6 and abs(self.y-gy) <= 1.2:
                self.twist.linear.y = 0.3*abs(self.y-gy)/(gy-self.y)
            if abs(self.y-gy) > 1.2 and abs(self.y-gy) <= 1.8:
                self.twist.linear.y = 0.4*abs(self.y-gy)/(gy-self.y)
            if abs(self.y-gy) > 1.8 and abs(self.y-gy) <= 2.4:
                self.twist.linear.y = 0.5*abs(self.y-gy)/(gy-self.y)
            self.cmd_vel_pub.publish(self.twist)

            if(abs(self.x-gx) < 0.02 and abs(self.y-gy) < 0.02):
                break
            print self.twist.linear.x, self.twist.linear.y

    def image_callback(self, msg):
        c = 1


'''
follower.fixed(gx=1.2, gy=0.3)
print("1")

follower.fixed(gx=0.0, gy=0.0)
print("2")
'''

# 定义状态

# 走到二维码识别区


class goto_QRcode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state goto_QRcode')
        follower.fixed(gx=1.2, gy=0.3)
        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'


# 读取二维码


class read_QRcode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state read_QRcode')
        self.flag = int(input('输入0或1：'))
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
        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# 抓取上层三种颜色的原料


class grab_material_upper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state grab_material_upper')
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
        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'


# 将原料放在粗加工区


class put_material_down_area1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state put_material_down_area1')
        self.flag = int(input('输入0或1：'))
        if self.flag == 1:
            return 'succeeded'
        elif self.flag == 0:
            return 'failed'

# 抓取粗加工区域的原料


class grab_processed_material(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
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
    rospy.init_node('smach_example_state_machine')
    # 创建一个状态机
    sm = smach.StateMachine(outcomes=['outcome1', 'outcome2'])
    # 打开状态机容器
    with sm:
        # 使用add方法添加状态到状态机容器当中
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

    add_thread = threading.Thread(target=th)
    add_thread.start()
    main()


