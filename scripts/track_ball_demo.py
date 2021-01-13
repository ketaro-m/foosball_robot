#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# parameters you need to fill in depending on the camera setting
field_area = [[37, 55], [605, 405]] # [[top-left x,y], [bottom-right x, y]]
# obtain from hsv.py
hsv_lower = np.array([20, -10, 100])
hsv_upper = np.array([50, 64, 300])
median_size = 7      # filter size for median filter
morpho_size = 13     # filter size for morphology processing


field_contour = [field_area[0], 
            [field_area[0][0],
            field_area[1][1]], 
            field_area[1], 
            [field_area[1][0], field_area[0][1]]]

class cvBridgeDemo:
    def __init__(self):
        global field_contour
        self.field = field_contour
        self.stencil_flag = False # not to make stencil more than once
        self.centers = []
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_rect_color", Image, self.image_callback, queue_size=1)

    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        input_image = np.array(frame, dtype=np.uint8)

        self.process_image(input_image, True)

        print(self.circles)
        cv2.waitKey(1)

    def process_image(self, image, debug=False):
        # hsv filter
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        if (not self.stencil_flag):
            self.stencil_flag = True
            self.stencil = np.zeros(mask.shape).astype(mask.dtype)
            cv2.fillConvexPoly(self.stencil, np.array(self.field), [255, 255, 255])
        mask = cv2.bitwise_and(mask, self.stencil)

        if debug:
            display = cv2.bitwise_and(image, image, mask= mask)
            cv2.imshow("hsv filter", display)   

        global median_size
        mask = cv2.medianBlur(mask,median_size)
        # morphology processing
        global morpho_size
        kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(morpho_size,morpho_size))
        mask = cv2.dilate(mask,kernel,iterations = 1)
        mask = cv2.erode(mask,kernel,iterations = 1)

        if debug:
            display = cv2.bitwise_and(image, image, mask= mask)
            cv2.imshow("morphology processing", display)   

        # make contour
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if (len(contours) == 0):
            if debug:
                display = image.copy()
                cv2.imshow("ball region", display)   
            return
        
        max_cnt = max(contours, key=lambda x: cv2.contourArea(x))
        out = np.zeros_like(mask)
        cv2.drawContours(out, [max_cnt], -1, color=255, thickness=-1)
        # cv2.imshow("out", out)
        mask = out
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if debug:
            display = cv2.bitwise_and(image, image, mask= mask)
            cv2.imshow("largest area", display)   
        
        # if debug:
        #     display = np.zeros(mask.shape, dtype=np.uint8)
        #     for c in contours:
        #         for elem in c:
        #             display[elem[0,1],elem[0,0]]=255
        #     cv2.imshow("make contours", display)   

        # make region
        circles = []
        for contour in contours:
            (x,y),radius = cv2.minEnclosingCircle(contour)
            center = (int(x),int(y))
            radius = int(radius)
            circles.append({"center":center, "radius":radius})

        if debug:
            display=image.copy()
            cv2.rectangle(display, tuple(field_area[0]),tuple(field_area[1]), (0, 255, 0))
            for i in range(5):
                x = (field_area[1][0]-field_area[0][0])/6*(i+1) + field_area[0][0]
                cv2.line(display,(x,field_area[0][1]),(x,field_area[1][1]),(0,255,0))
            for i in range(2):
                y = (field_area[1][1]-field_area[0][1])/3*(i+1) + field_area[0][1]
                cv2.line(display,(field_area[0][0],y),(field_area[1][0],y),(0,255,0))
            for circle in circles:
                cv2.circle(display,circle["center"],circle["radius"],(0,0,255),2)
            cv2.imshow("ball region", display)   

        self.circles = circles

    def cleanup(self):
        cv2.destroyAllWindows()   

if __name__ == '__main__':
    cvBridgeDemo()
    rospy.spin()



