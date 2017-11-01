#Source: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import time
import argparse

import color
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt


class image_converter:
    def __init__(self, demo):
        image_topic = "/cameras/left_hand_camera/image"
        self.image_pub = rospy.Publisher("/robot/xdisplay", Image, latch=True, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)
        self.cd = color.color_detector()
        self.position = ()
        self.distance = {}
        self.msg = np.array([])
        self.lastImage = np.array([])
        self.demo = demo
        self.lastCall = 0
        self.sampleRate = 0

    def display_image(self):
        """
            Displays image on baxter head and in window on computer
        """
        print('Publishing to display')
        try:
            cv2.imshow("images", self.msg)
            cv2.imwrite("finalLoc.png", self.msg)
            cv2.waitKey(0)
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
            cv2.imshow("images", self.lastImage)
            cv2.imwrite("start.png", self.lastImage)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)
        if self.msg.shape[0] > 0:
            self.msg = self.lastImage
            self.display_image()
        if self.demo and time.time() - self.lastCall > self.sampleRate:
            print("Getting domino..")
            self.lastCall = time.time()
            self.getNextDomino()
            self.sampleRate = time.time() - self.lastCall

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
                print("Domino found at", self.position)
            else:
                print("No dominoes found")

        return self.position

def main():
    parser = argparse.ArgumentParser(description="Identifies dominoes and relative position to origin")
    demo = False
    parser.add_argument('-d', nargs='?', const=True, default=demo, help="Run Demo mode")
    args = parser.parse_args()
    ic = image_converter(args.d)
    print("Initialising...")
    rospy.init_node('image_converter', anonymous=True\
    try:
        rospy.spin()
        for i in range(10):
            cv2.waitKey(0)
            print("Displaying...")
            ic.getNextDomino()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)