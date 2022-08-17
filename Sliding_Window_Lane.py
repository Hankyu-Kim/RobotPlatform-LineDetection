#! /usr/bin/env python
# coding=UTF-8
import rospy
from sensor_msgs.msg import Image, CompressedImage 
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError # rosimage -> opencv image
import cv2
import numpy as np



class line_tracing():
    def __init__(self):        
        # self.masked_img_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        self.webcam_img_subscriber = rospy.Subscriber("/camera/rgb/image_rect_color/compressed", CompressedImage, self.callback)
        # self.pub = rospy.Publisher('/filtered_image', CompressedImage, queue_size=5)
        # self.pub1 = rospy.Publisher('/test', String, queue_size=5)
        self.pub2 = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.bridge = CvBridge() # CvBridge 
        self.initialized=False

        self.lower_lane=np.array([0, 143, 104]) #camera_callback variables
        self.upper_lane =np.array([138, 255, 255])

        self.topleft = [173, 273] #wrapping variables
        self.topright = [196, 273]
        self.bottomleft = [42, 432]
        self.bottomright = [422,432]
        self.topleft2 = [0, 0]
        self.topright2 = [200, 0]
        self.bottomleft2 = [200, 432]
        self.bottomright2 = [640, 432]
        self.source = np.float32([self.topleft, self.topright, self.bottomleft, self.bottomright])
        self.destination = np.float32([self.topleft2, self.topright2, self.bottomleft2, self.bottomright2])

    def wrapping(self, image):
        (h, w) = (image.shape[0], image.shape[1])

        transform_matrix = cv2.getPerspectiveTransform(self.source, self.destination)
        minv = cv2.getPerspectiveTransform(self.destination, self.source)
        _image = cv2.warpPerspective(image, transform_matrix, (w, h))

        return _image, minv

    def camera_callback(self, image): 

        image = image[360:1080, 0:360]
    
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

    def roi(self, image):
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

    def plothistogram(self, image):
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

    # def how_much_curved(self, draw_info):
    #     left_fitx = draw_info['left_fitx']
    #     ploty = draw_info['ploty']

    #     mean_x = np.mean((left_fitx, right_fitx), axis=0)
    #     pts_mean = np.flipud(np.transpose(np.vstack([mean_x, ploty])))

    #     return pts_mean

    def move(self, ltx, right_lane_x):
        msg = Twist()
        gradient = round((-ltx[0]+160)/320, 1)
        # rospy.loginfo(ltx[0])

        msg.linear.x=0.5
        # msg.angular.z=0.2
        # msg.angular.z=(-right_lane_x+115)/50 #dot_tracing
        msg.angular.z=gradient #line_tracing
        # rospy.loginfo((-gradient+280)/100)
        self.pub2.publish(msg)

    def callback(self, _data):

        img = self.bridge.compressed_imgmsg_to_cv2(_data, desired_encoding='passthrough')

        w_f_img, right_lane_x = self.camera_callback(img)

        # w_f_r_img = self.roi(w_f_img)

        # wrapped_img, minverse = self.wrapping(w_f_img)
        
        ret, thresh = cv2.threshold(w_f_img, 160, 255, cv2.THRESH_BINARY)
        
        cv2.imshow('test', w_f_img)

        leftbase, rightbase = self.plothistogram(thresh)

        ltx = self.slide_window_search(thresh, leftbase, rightbase)
        
        # meanPts = self.how_much_curved(draw_info)
        # rospy.loginfo(meanPts)

        self.move(ltx, right_lane_x)

        # print('here')
        # self.pub1.publish('test')

        # try:
        #     self.pub.publish(self.bridge.cv2_to_imgmsg(w_f_img, "8UC1"))
        # except CvBridgeError as e:
        #     print(e)

def run():
    rospy.init_node("line_tracing_example")
    new_class = line_tracing()
    rospy.spin()
    

if __name__=='__main__':
    run()