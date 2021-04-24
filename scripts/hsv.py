#!/usr/bin/env python

"""
Program to obtain hsv_lower, hsv_upper values of the ball.
Prerequire to capture a camera image, and run with the image like
$ python hsv.py [img file]
"""
import cv2
import numpy as np

image_hsv = None   # global ;(
pixel = (20,60,80) # some stupid default
field_area = [[42, 67], [597, 410]] # [[top-left x,y], [bottom-right x, y]]
contours = [field_area[0], 
            [field_area[0][0],
            field_area[1][1]], 
            field_area[1], 
            [field_area[1][0], field_area[0][1]]]
stencil = None     # global

# mouse callback function
def pick_color(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
        pixel = image_hsv[y,x]

        #you might want to adjust the ranges(+-10, etc):
        upper =  np.array([pixel[0] + 30, pixel[1] + 30, pixel[2] + 40])
        lower =  np.array([pixel[0] - 30, pixel[1] - 30, pixel[2] - 40])
        print(pixel, lower, upper)

        image_mask = cv2.inRange(image_hsv,lower,upper)
        # cv2.imshow("mask",image_mask)
        result = cv2.bitwise_and(image_mask, stencil)
        cv2.imshow("mask_in_field",result)

def main():
    import sys
    global image_hsv, pixel # so we can use it in mouse callback

    image_src = cv2.imread(sys.argv[1])  # pick.py my.png
    if image_src is None:
        print ("the image read is None............")
        return    

    global stencil
    stencil = np.zeros(image_src.shape[:-1]).astype("uint8") # for mask
    cv2.fillConvexPoly(stencil, np.array(contours), [255, 255, 255])
    # result = cv2.bitwise_and(image_src, stencil)
    # cv2.imshow("in_field",result)


    # image_src = image_src[field_area[0][1] : field_area[1][1], field_area[0][0]: field_area[1][0]] # cropping

    ## NEW ##
    print("hsv, lower, upper")
    cv2.namedWindow('hsv')
    cv2.setMouseCallback('hsv', pick_color)

    # now click into the hsv img , and look at values:
    image_hsv = cv2.cvtColor(image_src,cv2.COLOR_BGR2HSV)

    cv2.rectangle(image_src, tuple(field_area[0]),tuple(field_area[1]), (0, 0, 255))
    cv2.imshow("bgr",image_src)
    cv2.rectangle(image_hsv, tuple(field_area[0]),tuple(field_area[1]), (0, 0, 255))
    cv2.imshow("hsv",image_hsv)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
