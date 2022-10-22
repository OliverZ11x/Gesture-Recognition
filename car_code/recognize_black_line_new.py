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
        image = cv2.resize(image, (480, 480))
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_black, upper_black)
        # BEGIN CROP
        h, w, d = image.shape
        # 缩小图像高度

        search_top = int(1*h/3)
        search_bot = search_top + 160
        mask[0:search_top, 0:w] = 0
    #        mask[search_bot:h, 0:w] = 0
        '''
        search_right = int(1*w/3)
        search_left = search_top + 160
        mask[0:h, 0:search_right] = 0
        mask[0:h, search_left:w] = 0
        '''

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
            '''
            7500-8500y
            17000-18000x
            20000z
            '''
            if area >18000  and area < 38000:
                ratio = box[3]-box[1]
                if abs(ratio[0]) > abs(ratio[1]):
                    #print("横线")
                    shap = 0
                else:
                    #print("竖线")
                    shap = 1
            elif area >= 38000:
                #print('交点')
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
            print(area, shap)
#            return shap

        cv2.imshow('mask', mask)
        cv2.imshow('image', image)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    
    cap.release()
    cv2.destroyAllWindows()

recognition()