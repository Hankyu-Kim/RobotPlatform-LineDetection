! / usr / bin / env
python2

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge  # rosimage -> opencv image
import numpy as np

import cv2


class CameraReceiver():
    def __init__(self):
        rospy.loginfo("Camera Receiver Object is Created")
        rospy.Subscriber("/camera/rgb/image_rect_color/Compressed", CompressedImage, self.camera_callback)
        self.bridge = CvBridge()  # CvBridge
        self.initialized = False

    def camera_callback(self, _data):
        if self.initialized == False:
            cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL)
            cv2.createTrackbar('low_H', 'Simulator_Image', 0, 255, nothing)
            cv2.createTrackbar('low_S', 'Simulator_Image', 0, 255, nothing)
            cv2.createTrackbar('low_V', 'Simulator_Image', 0, 255, nothing)
            cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
            self.initialized = True

        rospy.loginfo(len(_data.data))
        rospy.loginfo(type(_data.data))  # type string, length 54000
        cv_image = self.bridge.compressed_imgmsg_to_cv2(_data, desired_encoding='passthrough')
        rospy.loginfo(cv_image.shape)  # 0
        rospy.loginfo(type(cv_image))  # type numpy.ndarray, length 480, dim 3

        cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        crop_image = cv_image.copy()[240:480][:][:]
        cv2.imshow("crop_image", crop_image)
        cv2.line(crop_image, (300, 200), (640, 400), (255, 0, 0))

        low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
        low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
        low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
        high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
        high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
        high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')
        lower_lane = np.array([low_H, low_S, low_V])
        upper_lane = np.array([high_H, high_S, high_V])

        lane_image = cv2.inRange(crop_image, lower_lane, upper_lane)

        cv2.imshow("Simulator_Image", cv_image)
        cv2.imshow("lane_image", lane_image)

        channel_1, channel_2, channel_3 = cv2.split(cv_image)
        cv2.imshow("Simulator_Image, 1", channel_1)
        cv2.imshow("Simulator_Image, 2", channel_2)
        cv2.imshow("Simulator_Image, 3", channel_3)
        cv2.waitKey(1)  # waiting key 1ms


def nothing():
    pass


def run():
    rospy.init_node("camera_example")
    new_class = CameraReceiver()
    CameraReceiver.publish(Image)
    rospy.spin()


if __name__ == '__main__':
    run()