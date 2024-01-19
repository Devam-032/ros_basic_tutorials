#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math 
resultant,max = 0,0
def callback(data):
    global resultant,max
    ranges = [x for x in data.ranges]
    max=0
    for i in range(len(ranges)):
        if not(math.isnan(ranges[i])):
            if (ranges[i]>3.5 or ranges[i]==0):
                ranges[i]=3.5
    for i in range(len(ranges)):
        if ranges[i] < 3.5:
            ranges[i] = 1-(ranges[i]/3.5)
        else:
            ranges[i] = 0
    for j in range(-50, 50):
        if (j >= 0):
            k = j
        else:
            k = len(ranges)+j
        resultant -= (j)*ranges[k]
        if (max < ranges[k]):
            max = ranges[k]
    print(max,k)
    
# def range_scan(ranges, i, j):
#     ranges = [x for x in ranges if not math.isnan(x)]
#     slice_of_array = ranges[i: j+1]
#     return ( sum(slice_of_array) / float(len(slice_of_array)) )
if __name__=='__main__':
    rospy.init_node('scan_node', anonymous=True)
    rospy.Subscriber('/scan',LaserScan,callback)
    rospy.spin()
