#! /usr/bin/env python2

import rospy
from sensor_msgs.msg import CompressedImage 
from cv_bridge import CvBridge # rosimage -> opencv image
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time





class line_tracing():
    def __init__(self):        
        # self.masked_img_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        self.webcam_img_subscriber = rospy.Subscriber("/camera/rgb/image_rect_color/compressed", CompressedImage, self.callback)
        
        self.bridge = CvBridge() # CvBridge 
        self.initialized=False

    def wrapping(self, image):
        (h, w) = (image.shape[0], image.shape[1])

        source = np.float32([[w // 2 - 60, h * 0.50], [w // 2 + 60, h * 0.50], [0, h], [w, h]])
        destination = np.float32([[0, 0], [w-350, 0], [400, h], [w-150, h]])

        transform_matrix = cv2.getPerspectiveTransform(source, destination)
        minv = cv2.getPerspectiveTransform(destination, source)
        _image = cv2.warpPerspective(image, transform_matrix, (w, h))

        return _image, minv

    def camera_callback(self, image): 

        if self.initialized == False:
            cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL)
            cv2.createTrackbar('low_H', 'Simulator_Image', 0, 255, nothing)
            cv2.createTrackbar('low_S', 'Simulator_Image', 0, 255, nothing)
            cv2.createTrackbar('low_V', 'Simulator_Image', 0, 255, nothing)
            cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
            self.initialized = True
            
        # rospy.loginfo(len(image.data)) #_data length is 54000
        # rospy.loginfo(type(image.data)) #_data is string type -> buffer
        # rospy.loginfo(type(image)) #type numpy.ndarray, length 480, dim 3
        
        #R= cv2.getTrackbarPos('R', 'Simulator_Image')
        #G=cv2.getTrackbarPos('G', 'Simulator_Image')
        #B=cv2.getTrackbarPos('B', 'Simulator_Image')
      
    
        cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # crop_image=cv_image.copy()[240:480][:][:]
        # cv2.imshow("crop_image", crop_image)
        # cv2.line(crop_image, (300, 200), (640, 400), (255, 0,0))
        
        low_H= cv2.getTrackbarPos('low_H', 'Simulator_Image')
        low_S=cv2.getTrackbarPos('low_S', 'Simulator_Image')
        low_V=cv2.getTrackbarPos('low_V', 'Simulator_Image')
        high_H= cv2.getTrackbarPos('high_H', 'Simulator_Image')
        high_S=cv2.getTrackbarPos('high_S', 'Simulator_Image')
        high_V=cv2.getTrackbarPos('high_V', 'Simulator_Image')
        lower_lane=np.array([low_H,low_S,low_V])
        upper_lane =np.array([high_H,high_S,high_V])
        
        
        lane_image = cv2.inRange(image, lower_lane, upper_lane) #final result
    
        cv2.imshow("Simulator_Image",image)
        cv2.imshow("lane_image",lane_image)
        
        channel_1, channel_2, channel_3= cv2.split(image)
        cv2.imshow("Simulator_Image, 1",channel_1)
        cv2.imshow("Simulator_Image, 2",channel_2)
        cv2.imshow("Simulator_Image, 3",channel_3)
        cv2.waitKey(1)

        return lane_image
    

    def roi(self, image):
        x = int(image.shape[1])
        y = int(image.shape[0])

        _shape = np.array(
            [[int(0.1*x), int(y)], [int(0.1*x), int(0.1*y)], [int(0.4*x), int(0.1*y)], [int(0.4*x), int(y)], [int(0.7*x), int(y)], [int(0.7*x), int(0.1*y)],[int(0.9*x), int(0.1*y)], [int(0.9*x), int(y)], [int(0.2*x), int(y)]])

        mask = np.zeros_like(image)

        if len(image.shape) > 2:
            channel_count = image.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        cv2.fillPoly(mask, np.int32([_shape]), ignore_mask_color)
        masked_image = cv2.bitwise_and(image, mask)

        return masked_image

    def wrapped_img_threshold(self, w_f_r_img):
        _gray = cv2.cvtColor(w_f_r_img, cv2.COLOR_BGR2GRAY)

        # ret, thresh = output 1, 2

        return cv2.threshold(_gray, 160, 255, cv2.THRESH_BINARY)


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
        right_lane = []
        color = [0, 255, 0]
        thickness = 2

        for w in range(nwindows):
            win_y_low = binary_warped.shape[0] - (w + 1) * window_height
            win_y_high = binary_warped.shape[0] - w * window_height
            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin
            win_xright_low = right_current - margin
            win_xright_high = right_current + margin 

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), color, thickness)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), color, thickness)
            good_left = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]
            left_lane.append(good_left)
            right_lane.append(good_right)
            # cv2.imshow("oo", out_img)

            if len(good_left) > minpix:
                left_current = int(np.mean(nonzero_x[good_left]))
            if len(good_right) > minpix:
                right_current = int(np.mean(nonzero_x[good_right]))

        left_lane = np.concatenate(left_lane)
        right_lane = np.concatenate(right_lane)

        leftx = nonzero_x[left_lane]
        lefty = nonzero_y[left_lane]
        rightx = nonzero_x[right_lane]
        righty = nonzero_y[right_lane]

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        ltx = np.trunc(left_fitx)
        rtx = np.trunc(right_fitx)

        out_img[nonzero_y[left_lane], nonzero_x[left_lane]] = [255, 0, 0]
        out_img[nonzero_y[right_lane], nonzero_x[right_lane]] = [0, 0, 255]

        # plt.imshow(out_img)
        # plt.plot(left_fitx, ploty, color = 'yellow')
        # plt.plot(right_fitx, ploty, color = 'yellow')
        # plt.xlim(0, 1280)
        # plt.ylim(720, 0)
        # plt.show()

        ret = {'left_fitx' : ltx, 'right_fitx': rtx, 'ploty': ploty}

        return ret

    def how_much_curved(self, draw_info):
        left_fitx = draw_info['left_fitx']
        right_fitx = draw_info['right_fitx']
        ploty = draw_info['ploty']

        mean_x = np.mean((left_fitx, right_fitx), axis=0)
        pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])



        return pts_mean

    def callback(self, _data):

        img = self.bridge.compressed_imgmsg_to_cv2(_data, desired_encoding='passthrough')

        wrapped_img, minverse = self.wrapping(img)
        # cv2.imshow('wrapped', wrapped_img)

        w_f_img = self.camera_callback(wrapped_img)
        # cv2.imshow('w_f_img', w_f_img)

        w_f_r_img = self.roi(w_f_img)
        # cv2.imshow('w_f_r_img', w_f_r_img)

        # _gray = cv2.cvtColor(w_f_r_img, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(w_f_r_img, 160, 255, cv2.THRESH_BINARY)
        # cv2.imshow('threshold', thresh)

        leftbase, rightbase = self.plothistogram(thresh)
        # plt.plot(hist)
        # plt.show()

        draw_info = self.slide_window_search(thresh, leftbase, rightbase)
        # plt.plot(left_fit)
        # plt.show()

        meanPts = self.how_much_curved(draw_info)
        rospy.loginfo(meanPts)


def nothing():
    pass       

def run():
    rospy.init_node("line_tracing_example")
    new_class = line_tracing()
    rospy.spin()
    

if __name__=='__main__':
    run()