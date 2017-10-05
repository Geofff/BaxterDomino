#Source: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

from __future__ import print_function

import cv2
import sys
import roslib
import rospy
import edge

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt





class image_converter:

    def __init__(self, image_topic):
        self.image_pub = rospy.Publisher("/robot/xdisplay", Image, latch=True, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)
        self.edger = edge.edge_detector()

    def display_image(self, msg):
        print('Publishing to display')
        self.image_pub.publish(msg)
        rospy.sleep(3)

    def callback(self, data):
        print("Calling.")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)
        print("Obtained image..")
        cv_edge = self.edger.getEdges(cv_image, 500, 1000)
        print("Found Edges")
        plt.subplot(121), plt.imshow(cv_image, cmap="gray")
        plt.title("Current View"), plt.xticks([]), plt.yticks([])
        plt.subplot(122), plt.imshow(cv_edge, cmap="binary")
        plt.title("Edges"), plt.xticks([]), plt.yticks([])
        plt.show()
        #cv2.imshow("Image Window", cv_image)
        rospy.sleep(2)
        try:
            msg = self.bridge.cv2_to_imgmsg(cv_image, "mono8")
            self.display_image(msg)
        except CvBridgeError as e:
            print(e)

"""def setup_cameras():
    right = baxter_interface.CameraController("right_hand_camera")
    right.close()
    head = baxter_interface.CameraController("head_camera")
    head.resolution = (640,400)
    head.open()"""


def main(args):
    #setup_cameras()
    image_topic = "/cameras/right_hand_camera/image"
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