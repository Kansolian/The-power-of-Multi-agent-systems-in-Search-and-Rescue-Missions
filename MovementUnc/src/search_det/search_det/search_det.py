import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from sensor_msgs.msg import Image
import math
import numpy as np
import time
import random 
import cv2
from cv_bridge import CvBridge


import numpy as np






class Detection(Node):
    def __init__(self):
        # Image initialization
        super().__init__('detection')

        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value


        self.br = CvBridge()
        self.sub = self.create_subscription(
            Image, robot_prefix+'/camera/image_raw',self.image_subscriber_callback, 10)
        

    def image_subscriber_callback(self, msgs):
        current_frame = self.br.imgmsg_to_cv2(msgs)

        # Convert BGR to HSV colorspace
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        hsvFrame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        #blue color
        blue_lower = np.array([94, 80, 2], np.uint8)
        blue_upper = np.array([120, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

        # Blue detection
        kernal = np.ones((5, 5), "uint8")
        blue_mask = cv2.dilate(blue_mask, kernal)
        res_blue = cv2.bitwise_and(current_frame, current_frame, mask=blue_mask)

        # Track blue color
        contours, hierarchy = cv2.findContours(blue_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                current_frame = cv2.rectangle(current_frame, (x, y),
                                        (x + w, y + h),
                                        (255, 0, 0), 2)

                cv2.putText(current_frame, "Blue", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 0, 0))


        cv2.imshow("Color Detection", current_frame)
        cv2.waitKey(1)


        
def main(args=None):
    rclpy.init(args=args)

    detection = Detection()
    rclpy.spin(detection)
    Detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()