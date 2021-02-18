from styx_msgs.msg import TrafficLight
import cv2
import rospy
import numpy as np


class TLClassifier(object):
    def __init__(self):
        self.cv_image = None
        self.crop_image = None
        self.colors = [[0, 0, 255], [0, 255, 0], [255, 0, 0]]
        self.colorNames = ["red", "green", "blue"]

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        self.cv_image = image
        height, width, channel = self.cv_image.shape
        # rospy.logwarn("Heigh: {00000}, width: {00000}".format(height, width))
        self.crop_image = self.cv_image[30:300, 100:500]
        dst = self.crop_image.copy()
        gray = cv2.cvtColor(self.crop_image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, 1, 100, param1=250, param2=10, minRadius=10, maxRadius=80)

        if (circles.all()):
            for i in circles[0]:
                cv2.circle(dst, (i[0], i[1]), i[2], (255, 255, 255), 2)
                cv2.imwrite('test_cam.png', dst)

                B = dst[int(i[1]), int(i[0])][0]
                G = dst[int(i[1]), int(i[0])][1]
                R = dst[int(i[1]), int(i[0])][2]

                if (R - B) > 20 and (R - G) > 20 and (R) > 100:
                    rospy.logwarn("RED: B{00},G{00},R{00}".format(B, G, R))
                    return TrafficLight.RED

        return TrafficLight.UNKNOWN

        # TODO implement light color prediction
