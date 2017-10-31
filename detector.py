#Source: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

from __future__ import print_function

import roslib
import sys
import rospy
import cv2

import color
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt


class image_converter:
    def __init__(self):
        image_topic = "/cameras/left_hand_camera/image"
        self.image_pub = rospy.Publisher("/robot/xdisplay", Image, latch=True, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)
        self.cd = color.color_detector()
        self.position = ()
        self.distance = {}
        self.msg = np.array([])
        self.lastImage = np.array([])
        self.active = True

    def display_image(self):
        print('Publishing to display')
        try:
            cv2.imshow("images", self.msg)
            msg = self.bridge.cv2_to_imgmsg(cv2.resize(self.msg, (1024, 600), interpolation=cv2.INTER_CUBIC), "bgr8")
            self.image_pub.publish(msg)
            rospy.sleep(1)
        except CvBridgeError as e:
            print(e)

    def callback(self, data):
        """
            Processes message received from camera. only processes images after processing
            of previous message has completed. Throws frames in between.
        :param data image from camera
        """
        try:
            self.lastImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.msg.shape[0] > 0 and self.active:
            self.msg = self.lastImage
            self.display_image()

    def sensorCallback(self, msg, side):
        self.distance[side] = msg.range
        print("Sensor distance", side, msg.range)

    def getNextDomino(self):
        """
            Returns positon of next domino relative to origin
        :return tuple (positon in mm, angle in degree +CCW)
        """
        print("Next Domino...")
        self.position = ()
        if self.lastImage.shape[0] > 0:
            nextDomino = self.cd.toNextDomino(self.lastImage)
            if nextDomino:
                self.msg = nextDomino[3]
                if not self.position:
                    self.position = (nextDomino[0], nextDomino[1], nextDomino[2])
                self.display_image()
                self.msg = np.array([])
                print("Domino found at")
                print( self.position)
            else:
                print("No dominoes found")

        return self.position

def main(args):
    #setup_cameras()
    #rospy.init_node('image_listener')
    ic = image_converter()
    print("Initialising...")
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
        for i in range(10):
            print("Displaying...")
            ic.getNextDomino()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
