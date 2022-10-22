from cv2 import cv2
import numpy


def recognition():
    cap = cv2.VideoCapture(0)
    while True:
        lower_black = numpy.array([0,  0,  0])
        upper_black = numpy.array([180, 255, 60])
        _, image = cap.read()
        # 重设图片尺寸以提高计算速度
        # frame = imutils.resize(frame, width=600)
        '''
        smg = rospy.wait_for_message('/camera', Image, timeout=30)
        image = bridge.imgmsg_to_cv2(smg, desired_encoding='bgr8')
        '''
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
    # 480 640 3
        '''
        屏幕中点图像，需要沿此中点图像寻线
        '''
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

        
        # 缩小图像宽度

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
            rect = cv2.minAreaRect(c)  # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
            # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
            box = cv2.boxPoints(rect)
            box = numpy.int0(box)
            # 画出来
            cv2.drawContours(image, [box], 0, (255, 0, 0), 1)
            # 计算轮廓面积
            x_area = cv2.contourArea(c)
        if len(x_cnts_right) > 0:
            # 找到面积最大的轮廓
            c = max(x_cnts_right, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)  # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
            # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
            box = cv2.boxPoints(rect)
            box = numpy.int0(box)
            # 画出来
            cv2.drawContours(image, [box], 0, (255, 0, 0), 1)
            # 计算轮廓面积
            x_area_right = cv2.contourArea(c)
        if len(x_cnts_left) > 0:
            # 找到面积最大的轮廓
            c = max(x_cnts_left, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)  # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
            # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
            box = cv2.boxPoints(rect)
            box = numpy.int0(box)
            # 画出来
            cv2.drawContours(image, [box], 0, (255, 0, 0), 1)
            # 计算轮廓面积
            x_area_left = cv2.contourArea(c)



        y_area=1
        y_area_right=1
        y_area_left=1
        if len(y_cnts) > 0:
            # 找到面积最大的轮廓
            c = max(y_cnts, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)  # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
            # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
            box = cv2.boxPoints(rect)
            box = numpy.int0(box)
            # 画出来
            cv2.drawContours(image, [box], 0, (255, 0, 0), 1)
            # 计算轮廓面积
            y_area = cv2.contourArea(c)
        if len(y_cnts_right) > 0:
            # 找到面积最大的轮廓
            c = max(y_cnts_right, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)  # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
            # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
            box = cv2.boxPoints(rect)
            box = numpy.int0(box)
            # 画出来
            cv2.drawContours(image, [box], 0, (255, 0, 0), 1)
            # 计算轮廓面积
            y_area_right = cv2.contourArea(c)
        if len(y_cnts_left) > 0:
            # 找到面积最大的轮廓
            c = max(y_cnts_left, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)  # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
            # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
            box = cv2.boxPoints(rect)
            box = numpy.int0(box)
            # 画出来
            cv2.drawContours(image, [box], 0, (255, 0, 0), 1)
            # 计算轮廓面积
            y_area_left = cv2.contourArea(c)

        
        print('x_area= ', x_area, '\tx_area_left= ', x_area_left, '\tx_area_right= ', x_area_right)
        #print('y_area= ', y_area, '\ty_area_left= ', y_area_left, '\ty_area_right= ', y_area_right)

        cv2.imshow('x', x)
        cv2.imshow('x_right', x_right)
        cv2.imshow('x_left', x_left)

        cv2.imshow('y', y)
        cv2.imshow('y_right', y_right)
        cv2.imshow('y_left', y_left)

        
        #cv2.imshow('x', x)
        cv2.imshow('image', image)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


recognition()
