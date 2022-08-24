#! /usr/bin/env python
# coding=UTF-8

import rospy # to subscribe and publish topic
from sensor_msgs.msg import CompressedImage # to lower the size of data
# from std_msgs.msg import String # this is for check 
from geometry_msgs.msg import Twist # topic type is Twist which is including vector 
from cv_bridge import CvBridge # rosimage -> opencv image
import cv2 # to deal with video (=computer vision)
import numpy as np # to deal with detail pixel, we used numpy array
from sensor_msgs.msg import LaserScan
import math
import time

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
you have to activate camera and limo command mode setting
1.  roslaunch astra_camera dabai_u3.launch 
(another terminal)
2.  roslaunch limo_bringup limo_start.launch
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

class line_tracing():
    def __init__(self): # to set initial value
        self.sub_ls = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color/compressed", CompressedImage, self.callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.bridge = CvBridge()

        self.mode = True

        self.lower_lane=np.array([0, 143, 104]) #HSV_filter variables
        self.upper_lane =np.array([138, 255, 255])

        self.topleft = [260, 273] #wrapping variables
        self.topright = [295, 273]
        self.bottomleft = [64, 432]
        self.bottomright = [633,432]
        self.topleft2 = [0, 0]
        self.topright2 = [200, 0]
        self.bottomleft2 = [200, 432]
        self.bottomright2 = [640, 432]
        self.source = np.float32([self.topleft, self.topright, self.bottomleft, self.bottomright])
        self.destination = np.float32([self.topleft2, self.topright2, self.bottomleft2, self.bottomright2])

    def lidar_callback(self, _data):
        '''
            실제 라이다 데이터를 받아서 동작하는 부분
            라이다가 측정되는 각도 계산(Radian 및 Degree)
            측정된 데이터 중, 위험 범위 안에 들어 있는 점의 수를 카운팅
            카운팅된 점의 수가 기준 이상이면, 위험 메시지 전달
        '''
        cnt = 0
        angle_rad = [_data.angle_min + i * _data.angle_increment for i, _ in enumerate(_data.ranges)]
        angle_deg = [180 / math.pi * angle for angle in angle_rad]
        for i, angle in enumerate(angle_deg):
            if -1 <= angle <= 1 and 0.0 < _data.ranges[i] < 1.0:
                cnt += 1
            if cnt >= 2:
                self.mode = False
            else:
                self.mode = True
    
    def wrapping(self, image): # we will not gonna use wrapping because of delay
        (h, w) = (image.shape[0], image.shape[1])

        #cv2.getPerspectiveTransform take long time & make delay
        transform_matrix = cv2.getPerspectiveTransform(self.source, self.destination)
        minv = cv2.getPerspectiveTransform(self.destination, self.source)
        _image = cv2.warpPerspective(image, transform_matrix, (w, h))

        return _image, minv

    def HSV_filter(self, image): 

        image = image[360:1080, 0:360] # crop (=roi)
    
        cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lane_image = cv2.inRange(image, self.lower_lane, self.upper_lane)

        M = cv2.moments(lane_image)
        if M['m00'] != 0:
            self.x = int(M['m10']/M['m00'])
            self.y = int(M['m01']/M['m00'])
        else:
            self.x = 0
            self.y = 0
        cv2.circle(image, (self.x, self.y), 25, (0, 0, 0), -1)
        # cv2.imshow('image+dot', lane_image)
        cv2.waitKey(1)

        # rospy.loginfo(self.x)
        
        return lane_image, self.x

    def roi(self, image): # we will not gonna use roi because we will refer to the data right in front of us
        x = int(image.shape[1])
        y = int(image.shape[0])
        
        _shape = np.array(
        [[int(0.0*x), int(y)], [int(0.0*x), int(0.85*y)], [int(1.0*x), int(0.85*y)], [int(1.0*x), int(y)], [int(0.0*x), int(y)]])

        mask = np.zeros_like(image)

        ignore_mask_color = 255

        cv2.fillPoly(mask, np.int32([_shape]), ignore_mask_color)
        masked_image = cv2.bitwise_and(image, mask)
        # cv2.imshow("masked_image", masked_image)

        return masked_image

    def plothistogram(self, image): # to set for sliding window, analyze the distribution of 255
        histogram = np.sum(image[image.shape[0]//2:, :], axis=0)
        midpoint = int(histogram.shape[0]/2)
        leftbase = np.argmax(histogram[:midpoint])
        rightbase = np.argmax(histogram[midpoint:]) + midpoint

        return leftbase, rightbase

    def slide_window_search(self, binary_warped, left_current, right_current):
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))

        nwindows = 4
        window_height = int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()

        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])
        margin = 100
        minpix = 50
        left_lane = []
        color = [0, 255, 0]
        thickness = 2

        for w in range(nwindows):
            win_y_low = binary_warped.shape[0] - (w + 1) * window_height
            win_y_high = binary_warped.shape[0] - w * window_height
            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin


            # cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), color, thickness)
            # cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), color, thickness)
            good_left = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            left_lane.append(good_left)
            # cv2.imshow("oo", out_img)

            if len(good_left) > minpix:
                left_current = int(np.mean(nonzero_x[good_left]))

        left_lane = np.concatenate(left_lane)

        leftx = nonzero_x[left_lane]
        lefty = nonzero_y[left_lane]

        left_fit = np.polyfit(lefty, leftx, 2)

        ploty = np.linspace(0, binary_warped.shape[0] - 10, 2)
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]

        ploty = np.trunc(ploty)
        ltx = np.trunc(left_fitx)

        out_img[nonzero_y[left_lane], nonzero_x[left_lane]] = [255, 0, 0]

        # plt.imshow(out_img)
        # plt.plot(left_fitx, ploty, color = 'yellow')
        # plt.plot(right_fitx, ploty, color = 'yellow')
        # plt.xlim(0, 1280)
        # plt.ylim(720, 0)
        # plt.show()

        # ret = {'left_fitx' : ltx, 'ploty': ploty}

        return ltx

    def move(self, ltx, right_lane_x): # publish commend-velocity
        msg = Twist()
        gradient = round((-ltx[0]+160)/320, 1)

        msg.linear.x=0.5
        # msg.angular.z=1.0
        # msg.angular.z=(-right_lane_x+70)/50 # dot tracing
        # rospy.loginfo((right_lane_x-70)/50)
        msg.angular.z=gradient #line_tracing
        # rospy.loginfo(gradient)
        self.vel_pub.publish(msg)

    def stop(self):
        msg = Twist()
        msg.linear.x=0.0
        msg.angular.z=0.0
        self.vel_pub.publish(msg)
        # time.sleep(1)

    def callback(self, _data):

        img = self.bridge.compressed_imgmsg_to_cv2(_data, desired_encoding='passthrough')
        cv2.imshow('', img)
        # w_img, minverse = self.wrapping(img)

        w_f_img, right_lane_x = self.HSV_filter(img)

        # w_f_r_img = self.roi(w_f_img)
        
        ret, thresh = cv2.threshold(w_f_img, 160, 255, cv2.THRESH_BINARY)
        
        cv2.imshow('test', w_f_img)

        leftbase, rightbase = self.plothistogram(thresh)

        ltx = self.slide_window_search(thresh, leftbase, rightbase)

        if self.mode == True:
            self.move(ltx, right_lane_x)
        elif self.mode == False:
            self.stop()

def run():
    rospy.init_node("line_tracing_example")
    new_class = line_tracing()
    rospy.spin()
    

if __name__=='__main__':
    run()