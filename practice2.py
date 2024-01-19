#!/usr/bin/env python3
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(data):
    n=36
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    range1 = sector(n , data.ranges)
    rospy.loginfo(range1)
    min_value, min_index = min_range_index(range1)

    rospy.loginfo(min_value)
    rospy.loginfo(min_index)
    # if min_index > 180:
    #     min_index = -(360-min_index)
    # else:
    #     min_index = min_index
    # z = Twist()
    # try:
    #     k1 = (abs(90-abs(min_index)))/(90-abs(min_index))
    # except ZeroDivisionError as e:
    #     k1 = 0
    # try:
    #     k2 = abs(min_index)/min_index
    # except ZeroDivisionError as e:
    #     k2 = 0

    # vl = (180-(k1*k2*80*abs(min_index/180)))*k1
    # vr = (180+(k1*k2*80*abs(min_index/180)))*k1
    # z.linear.x = ((math.sqrt((vl**2)+(vr**2)))/700)*((min_value/1.5))
    # angular = math.degrees(math.atan2(vr, vl))+90
    # if angular > 180:
    #     angular = angular-360
    # else:
    #     angular = angular
    # z.angular.z = -(angular/200)*(.7-(min_value/1.5))
    # pub.publish(z)

def sector(n,ranges):
    range1 = []
    for i in range(0,n+1):
        print(i*360/n)
        range1 = range1.append(ranges[i*360/n:(i+1)*360/n]) 
    return range1


def min_range_index(range1):
    min1 , min_index = [] , []
    for x in range1:
        subrange = [y for y in x if not (math.isnan(y) or y == 0)]
        min1 = min1.append(min(subrange))
        min_index= min_index.append(subrange.index(min(subrange)))
    return min1 , min_index


if __name__ == '__main__':

    # init new a node and give it a name
    rospy.init_node('scan_node', anonymous=True)
    # subscribe to the topic /scan.
    rospy.Subscriber("scan", LaserScan, callback)
    rate = rospy.Rate(1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
