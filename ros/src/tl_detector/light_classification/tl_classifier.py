from styx_msgs.msg import TrafficLight
import cv2
import rospy
import numpy as np


class TLClassifier(object):
    def __init__(self):
        self.cv_image = None
        self.crop_image = None
        self.green_count = 0
        self.red_count = 0
        self.prev_traffic = TrafficLight.UNKNOWN
        self.color_name = ['RED', 'YELLOW', 'GREEN', '0', 'UNKNOWN']

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        self.cv_image = image
        height, width, channel = self.cv_image.shape
        self.crop_image = self.cv_image[100:550, 50:550]
        dst = self.crop_image.copy()
        gray = cv2.cvtColor(self.crop_image, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, 1, 100, param1=250, param2=10, minRadius=10, maxRadius=50)

        if (circles is not None):
            for i in circles[0]:
                cv2.circle(dst, (i[0], i[1]), i[2], (255, 255, 255), 2)
                B = dst[int(i[1]), int(i[0])][0]
                G = dst[int(i[1]), int(i[0])][1]+1
                R = dst[int(i[1]), int(i[0])][2]+2
                # rospy.logwarn("R {0}, G {1}, B {2}".format(R, G, B))

                if R > 220 and (R - B) > 40 and (R - G) > 40:
                    self.red_count += 2
                    cv2.imwrite(
                        ('cam_red' + str(self.red_count) + '.png'), dst)
                    if (self.red_count > 3):
                        self.green_count = 0
                        self.red_count = 4
                        self.prev_traffic = TrafficLight.RED

                elif G > 220 and (G - R) > 40 and (G - B) > 40:
                    self.green_count += 2
                    cv2.imwrite(
                        ('cam_green' + str(self.green_count) + '.png'), dst)
                    if (self.green_count > 3):
                        self.red_count = 0
                        self.green_count = 4
                        self.prev_traffic = TrafficLight.GREEN

            rospy.logwarn("Circles: R count {0}, G count {1}, light is {2}".format(
                self.red_count, self.green_count, self.color_name[self.prev_traffic]))
            return self.prev_traffic

        if (self.prev_traffic is not TrafficLight.UNKNOWN):
            if (self.red_count > 0):
                self.red_count -= 1

                if self.red_count < 1:
                    self.prev_traffic = TrafficLight.UNKNOWN

            elif (self.green_count > 0):
                self.green_count -= 1
                if self.green_count < 1:
                    self.prev_traffic = TrafficLight.UNKNOWN

            rospy.logwarn("Buffer : R count {0}, G count {1}, light is {2}".format(
                self.red_count, self.green_count, self.color_name[self.prev_traffic]))
            return self.prev_traffic

        rospy.logwarn("NoCircl: R count {0}, G count {1}, light is {2}".format(
            self.red_count, self.green_count, self.color_name[TrafficLight.UNKNOWN]))
        self.prev_traffic = TrafficLight.UNKNOWN
        return self.prev_traffic
