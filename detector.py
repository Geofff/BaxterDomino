#Source: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import edge

import color
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt


class image_converter:
    def __init__(self, image_topic):
        self.image_pub = rospy.Publisher("/robot/xdisplay", Image, latch=True, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)
        self.cd = color.color_detector()
        self.ed = edge.edge_detector()

    def display_image(self, msg):
        print('Publishing to display')
        self.image_pub.publish(msg)
        rospy.sleep(3)

    def callback(self, data):
        print("Calling.")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        print("Obtained image..")
        edges = self.ed.getEdgesF(cv_image, 1000, 5000)
        edgeImage = self.ed.mergeEdges(cv_image, edges)
        print("Display Edges...")
        #cv2.imshow("images", np.hstack([cv_image, edgeImage]))
        #cv2.waitKey(0)
        faces = self.cd.filterColors(cv_image)
        print("Located Dominoes: ", len(faces))
        for face in faces:
            print("Location: ", face[1], ", ", face[2], " Orientation: ", face[3])
            #edgeLength, lengthImage = self.cd.getLongEdgeLength(face)
            #print("Length: ", edgeLength)
        #cv2.waitKey(0)
        if faces:
            msg = self.cd.drawLocation(cv_image, (faces[0][1], faces[0][2]), faces[0][3])
        else:
            "No faces found"
            msg = cv_image
        try:
            cv2.imshow("images", msg)
            msg = self.bridge.cv2_to_imgmsg(cv2.resize(msg,(1024, 600), interpolation = cv2.INTER_CUBIC), "bgr8")
            self.display_image(msg)
            #cv2.waitKey(0)
        except CvBridgeError as e:
            print(e)


def main(args):
    #setup_cameras()
    #rospy.init_node('image_listener')
    image_topic = "/cameras/left_hand_camera/image"
    ic = image_converter(image_topic)
    print("Initialising...")
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)