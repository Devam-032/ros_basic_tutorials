#!/usr/bin/env python3
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(data):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    min_value, min_index,LEN , min_array_size= min_range_index(data.ranges)
    #rospy.loginfo(data.ranges)
    if min_index > 180:
        min_index = -(360-min_index)
    else:
        min_index = min_index
    z = Twist()
    print(min_array_size)
    """This is the Initial Logic"""
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
    """This is the 2nd logic"""
    # z.linear.x=min_value*0.25
    # if (min_index>180):
    #     min_index-=360
    # angular_vel=(min_index/min_value)/360
    # if(min_index>-20 or min_index<20):
    #     if(min_value<0.2):
    #         z.linear.x=-min_value*0.25
    #         z.angular.z=angular_vel*2
    # else:
    #     z.angular.z=-angular_vel
    # rate = rospy.Rate(100)
    # pub.publish(z)
    # rate.sleep()

def min_range_index(ranges):
    ranges = [x for x in ranges if not (math.isnan(x) or x == 0)]
    n=30
    y = [ranges[i:i + n] for i in range(0, len(ranges), n)]
    b=[min(y[1]),min(y[2]),min(y[3]),min(y[4]),min(y[5]),min(y[6]),min(y[7]),min(y[8]),min(y[9]),min(y[10]),min(y[11])]
    return (min(ranges), ranges.index(min(ranges)),len(ranges),b)



if __name__ == '__main__':
    rospy.init_node('scan_node', anonymous=True)
    # subscribe to the topic /scan.
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()
