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
        if (math.isnan(ranges[i]or ranges[i]==0)):
            if (ranges[i]>3.5):
                ranges[i]=3.5
            ranges[i]=1-(ranges[i]/3.5)
def calculator():
    global dist, x_cor, y_cor, z, pub,ranges,yaw
    z = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    z.linear.x = 0
    z.angular.z = 0
    resultant = 0
    dist = math.sqrt((x-x_cor)**2+(y-y_cor)**2)
    angle = math.atan2((y_cor-y), (x_cor-x))-yaw
    if (abs(angle) > 3.14):
        angle = angle-6.28*(abs(angle)/angle)
    g2g_linear = dist*(0.2)
    g2g_angular = angle*(0.4)
    if abs(g2g_angular) > .5:
        g2g_angular = .5*(abs(g2g_angular)/g2g_angular)
    if g2g_linear > 0.2:
        g2g_linear = 0.2

    #autonomous_controller
    z.linear.x=g2g_linear
    z.angular.z=g2g_angular
    z.linear.y=(g2g_linear*2+2*g2g_angular)*400
    z.linear.z=(-2*g2g_angular+2*g2g_linear)*400
    print(angle)
    # rospy.loginfo(z)
    # pub.publish(z)
    rospy.sleep(0.4)
    # '''resultant_calculator'''
    # max = 0
    # for i in range(len(ranges)):
    #     if ranges[i] < 3.5:
    #         ranges[i] = 1-(ranges[i]/3.5)
    #     else:
    #         ranges[i] = 0
    # for j in range(-60, 60):
    #     if (j >= 0):
    #         k = j
    #     else:
    #         k = 360+j
    #     resultant -= (j)*ranges[k]
    #     if (max < ranges[k]):
    #         max = ranges[k]
    # k_obs_linear = 3**(1-max)-1.5
    # # print(k_obs_linear)
    # k_obs_angular = resultant/(1000)
    # k_g2g_linear=2.5-(math.pow(2.5,k_obs_linear))
    # k_g2g_angular = (2.5-math.pow(2.5, k_obs_angular))
    # z.linear.x = k_obs_linear + k_g2g_linear*(g2g_linear)
    # # print(z.linear.x)
    # if (z.linear.x > 0.1):
    #     z.linear.x = 0.1
    # z.angular.z = k_obs_angular + k_g2g_angular*g2g_angular
    # if (abs(z.angular.z) > 0.2):
    #     z.angular.z = 0.2*abs(z.angular.z)/(z.angular.z)
    # z.linear.z=(2.9*z.angular.z+2*z.linear.x)*390
    # z.linear.y=(2*z.linear.x-2.9*z.angular.z)*390
    # pub.publish(z)
    # rospy.loginfo(z)
    # rospy.sleep(0.4)
        
def odometryCb(msg):
    global x, y, yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
rospy.init_node('scan_node', anonymous=True)
rospy.Subscriber("scan", LaserScan, callback)
rospy.Subscriber('odom', Odometry, odometryCb)
dist = 20
x_cor = float(input('Enter the x-coordinate of the goal:'))
y_cor = float(input('Enter the y-coordinate of the goal:'))
while not dist < 0.1 and not rospy.is_shutdown():
    calculator()
z.linear.x = 0
z.angular.z = 0
pub.publish(z)
rospy.spin()