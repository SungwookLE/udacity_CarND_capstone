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

        if (circles is not None):
            #circles = np.uint16(np.around(circles))
            for i in circles[0]:
                cv2.circle(dst, (i[0], i[1]), i[2], (255, 255, 255), 2)
                B = dst[int(i[1]), int(i[0])][0]
                G = dst[int(i[1]), int(i[0])][1]+1
                R = dst[int(i[1]), int(i[0])][2]+2
                #rospy.logwarn("R {0}, G {1}, B {2}".format(R, G, B))

                if (R) > 200 and (R - B) > 20 and (R - G) > 20:
                    self.red_count += 1

                    if (self.red_count > 3):
                        rospy.logwarn("RED STOP!, R {0}".format(R))
                        cv2.imwrite(
                            ('test_cam_red' + str(self.red_count) + '.png'), dst)
                        self.greed_count = 0
                    return TrafficLight.RED

                elif G > 200 and (G - R) > 20 and (G - B) > 20:
                    self.green_count += 1

                    if (self.green_count > 10):
                        rospy.logwarn("GREEN ON!, G {0}".format(G))
                        cv2.imwrite(
                            ('test_cam_green' + str(self.green_count) + '.png'), dst)
                        self.red_count = 0
                        return TrafficLight.GREEN

        return TrafficLight.UNKNOWN

        # TODO implement light color prediction
