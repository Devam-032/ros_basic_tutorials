#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

threshold = 0.8

def convert_to_binary(frame):

    original_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(original_image, (5, 5), 0)
    _, binary_image = cv2.threshold(blurred_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    normalized_image = binary_image.astype(np.uint8)
    return normalized_image


def multi_scale_template_matching(image, template):
    max=-1
    maxi_loc=-1
    max_scale=-1
    for scale in np.linspace(0.2,2.0,20):
        scaled_template = cv2.resize(template, None, fx=scale, fy=scale)
        result = cv2.matchTemplate(image, scaled_template, cv2.TM_CCOEFF_NORMED)
        _, max_value, _, max_loc = cv2.minMaxLoc(result)
        if max_value>=threshold and max_value>max:
            max=max_value
            maxi_loc=max_loc
            max_scale=scale

    return maxi_loc,max_scale


def callbackFunction(message):
    bridgeobject = CvBridge()
    rospy.loginfo('received a video message/frame')
    frame_back_to_cv = bridgeobject.imgmsg_to_cv2(message)
    result = convert_to_binary(frame_back_to_cv)
    right = cv2.imread("/home/rover/Downloads/right.jpg", cv2.IMREAD_GRAYSCALE)
    left = cv2.imread("/home/rover/Downloads/left.jpg", cv2.IMREAD_GRAYSCALE)
    
    right = right.astype(np.uint8)
    left = left.astype(np.uint8)
    match_loc,match_scale = multi_scale_template_matching(result, right)
    w=int(right.shape[1]*match_scale)
    h=int(left.shape[0]*match_scale)
    if match_loc!=-1:
        cv2.rectangle(frame_back_to_cv, match_loc, (match_loc[0] + w, match_loc[1] + h), (0, 255, 0), 2)
        print(math.degrees(math.atan((match_loc[0] - 320.0 + 0.5 * w)/match_loc[1])))
        direction='right'
        print(direction)

    match_loc,match_scale = multi_scale_template_matching(result, left)
    w=int(right.shape[1]*match_scale)
    h=int(left.shape[0]*match_scale)
    if match_loc!=-1:
        cv2.rectangle(frame_back_to_cv, match_loc, (match_loc[0] + w, match_loc[1] + h), (0, 255, 0), 2)
        print(math.degrees(math.atan((match_loc[0] - 320.0 + 0.5 * w)/match_loc[1])))
        direction='left'
        print(direction)
    
    
    cv2.imshow("camera", frame_back_to_cv)
    cv2.waitKey(1)

rospy.init_node('camera_sub', anonymous = True) 
rospy.Subscriber('Chatter', Image, callbackFunction)
rospy.spin()
cv2.destroyAllWindows()