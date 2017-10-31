#Source: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import time

import color
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt


class image_converter:
    def __init__(self):
        root_name = "/robot/range/"
        image_topic = "/cameras/left_hand_camera/image"
        sensor_name = ["left_hand_range/state", "right_hand_range/state"]
        self.image_pub = rospy.Publisher("/robot/xdisplay", Image, latch=True, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)
        self.cd = color.color_detector()
        self.position = ()
        self.distance = {}
        self.msg = []
        self.lastReceived = time.time()
        self.sampleRate = 0 #Seconds per frame
        #self.left_sensor = rospy.Subscriber(root_name + sensor_name[0], Range,
        #                    callback=self.sensorCallback, callback_args="left", queue_size=1)
        #self.right_sensor = rospy.Subscriber(root_name + sensor_name[1], Range,
        #                                   callback=self.sensorCallback, callback_args="right", queue_size=1)

    def display_image(self):
        print('Publishing to display')
        try:
            cv2.imshow("images", self.msg)
            cv2.waitKey(1)
            msg = self.bridge.cv2_to_imgmsg(cv2.resize(self.msg, (1024, 600), interpolation=cv2.INTER_CUBIC), "bgr8")
            self.image_pub.publish(msg)
            rospy.sleep(1)
        except CvBridgeError as e:
            print(e)

    def callback(self, data):

        if time.time() - self.lastReceived > self.sampleRate:
            self.lastReceived = time.time()
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
            #print("Obtained image..")
            #print("Display Edges...")
            #cv2.imshow("images", np.hstack([cv_image, edgeImage]))
            #cv2.waitKey(0)
            nextDomino = self.cd.toNextDomino(cv_image)
            #self.cd.getAllDominoes(cv_image)
            if nextDomino:
                #cv2.imshow("images", np.hstack([cv_image, nextDomino[2]]))
                #cv2.waitKey(0)
                self.msg = nextDomino[3]
                if not self.position:
                    print("Next Domino...")
                    self.position = (nextDomino[0], nextDomino[1], nextDomino[2])
                #print("Position: ",self.position)
            else:
                print("No faces found")
                self.msg = cv_image
            self.display_image()
            self.sampleRate = time.time() - self.lastReceived


    def sensorCallback(self, msg, side):
        self.distance[side] = msg.range
        print("Sensor distance", side, msg.range)

    def getNextDomino(self):
        """
            Returns positon of next domino relative to origin
        :return tuple (positon in mm, angle in degree +CCW)
        """
        position = self.position
        self.position = ()
        return position

def main(args):
    #setup_cameras()
    #rospy.init_node('image_listener')
    ic = image_converter()
    print("Initialising...")
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)