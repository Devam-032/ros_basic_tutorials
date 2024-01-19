#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
x, y = 0, 0


def callback(data):
    global x, y, yaw, dist, x_cor, y_cor, ranges
    ranges = [x for x in data.ranges if not (math.isnan(x) or x == 0)]


def calculator(a):
    global dist, x_cor, y_cor, z, pub
    z = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    dist = 20
    z.linear.x = 0
    z.angular.z = 0
    dist = 20
    if (a == 2):
        resultant = 0
        dist = math.sqrt((x-x_cor)**2+(y-y_cor)**2)
        angle = math.atan2((y_cor-y), (x_cor-x))-yaw
        if (abs(angle) > 3.14):
            angle = angle-6.28*(abs(angle)/angle)
        g2g_linear = dist*(0.2)
        g2g_angular = angle*(0.4)
        if g2g_angular > 0.7:
            g2g_angular = 0.7
        if g2g_linear > 0.7:
            g2g_linear = 0.7
        '''resultant_calculator'''
        check = math.floor(yaw*180/(math.pi))
        max = 0
        for i in range(len(ranges)):
            if ranges[i] < 3.5:
                ranges[i] = 1-(ranges[i]/3.5)
            else:
                ranges[i] = 0
        for j in range(check-80, check+80):
            if (j >= 0):
                k = j
            else:
                k = 360+j
            resultant -= (j-check)*ranges[k]
            if (max < ranges[k]):
                max = ranges[k]
        k_obs_linear = 1-max
        k_obs_angular = resultant/(700)
        # if (k_obs_linear > 0.5):
        #     k_g2g_linear = (k_obs_linear)*(0.5)
        # else:
        #     k_g2g_linear = (k_obs_linear)*(0.5)+0.25
        k_g2g_linear=2.5-(math.pow(2.5,k_obs_linear))
        # if (k_obs_angular > 0.5):
        #     k_g2g_angular = (k_obs_angular)*(0.25)
        # else:
        #     k_g2g_angular = 1-(k_obs_angular)+0.55
        k_g2g_angular=(2.5-math.pow(2.5,k_obs_angular))
        z.linear.x = k_obs_linear + k_g2g_linear*(g2g_linear)
        if (z.linear.x > 0.4):
            z.linear.x = 0.4
        z.angular.z = k_obs_angular + k_g2g_angular*g2g_angular
        rospy.loginfo(z)
        if (abs(z.angular.z) > 0.6):
            z.angular.z = 0.6*abs(z.angular.z)/(z.angular.z)
        pub.publish(z)
        rospy.sleep(0.4)


def odometryCb(msg):
    global x, y, yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


if __name__ == '__main__':
    rospy.init_node('scan_node', anonymous=True)
    # subscribe to the topic /scan.
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.Subscriber('odom', Odometry, odometryCb)
    i = 2
    dist = 20
    x_cor = float(input('Enter the x-coordinate of the goal:'))
    y_cor = float(input('Enter the y-coordinate of the goal:'))
    while not dist < 0.15 and not rospy.is_shutdown():
        calculator(i)
    z.linear.x = 0
    z.angular.z = 0
    pub.publish(z)
    rospy.spin()
x=0
if(x==0):
    x='d'
    print(x)