#!/usr/bin/env python3
import rospy, math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
x, y = 0, 0

def callback(data):
    global x, y, yaw, dist, x_cor, y_cor, ranges
    ranges = [x for x in data.ranges]
    for i in range(len(ranges)):
        if (math.isnan(ranges[i])or ranges[i]==0):
                ranges[i]=3.5

def calculator():
    global dist, x_cor, y_cor, z, pub,ranges
    z = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    dist = 20
    z.linear.x = 0
    z.angular.z = 0
    sum=0
    resultant=0
    min=0
    '''resultant_calculator'''
    for i in range(len(ranges)):
        if ranges[i] < 3.5:
            ranges[i] = 1-(ranges[i]/3.5)
        else:
            ranges[i] = 0
    for j in range(-45, 45):
        if (j >= 0):
            k = j
        else:
            k = 260+j

        resultant -= (j)*ranges[k]
        sum=sum+ranges[k]
        if(min<ranges[k]):
            min=ranges[k]
    z.angular.z = resultant/(2000)
    if(min<.9):
        z.linear.x = (120-sum)/120
    else:
        z.linear.x= -.25
    if (z.linear.x > 0.3):
        z.linear.x = 0.3
    if (abs(z.angular.z) > 0.3):
        z.angular.z = 0.3*abs(z.angular.z)/(z.angular.z)
    if(z.angular.z>0.2):
        z.linear.y=205
        z.linear.z=-205
    elif(.05<z.angular.z<.2):
        z.linear.y=250
        z.linear.z=-100
    elif(-.05>z.angular.z>-.2):
        z.linear.y=-100
        z.linear.z=250
    elif(z.angular.z<-.2):
        z.linear.y=-205
        z.linear.z=205
    else:
        z.linear.z=200
        z.linear.y=200
    if(z.linear.x<0):
        z.linear.z=-200
        z.linear.y=-200
    print(z.linear.y,z.linear.z)
         
    pub.publish(z)
    rospy.sleep(0.4)
rospy.init_node('scan_node', anonymous=True)
rospy.Subscriber("scan", LaserScan, callback)
# rospy.Subscriber('odom', Odometry, odometryCb)
dist = 20
x_cor = float(input('Enter the x-coordinate of the goal:'))
y_cor = float(input('Enter the y-coordinate of the goal:'))
while not dist < 0.1 and not rospy.is_shutdown():
    calculator()
pub.publish(z)
rospy.spin()