#!/usr/bin/env python3
import rospy, math
from sensor_msgs.msg import LaserScan,Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
x, y = 0, 0

def callback(data):
    global x, y, yaw, dist, x_cor, y_cor, ranges
    ranges = [x for x in data.ranges]
    for i in range(len(ranges)):
        if not (math.isnan(ranges[i])):
            if (ranges[i]>3.5 or ranges[i]==0):
                ranges[i]=3.5
            print(ranges)

def calculator():
    global dist, x_cor, y_cor, z, pub,ranges,angle,distance,yaw
    z = Twist()
    dist=distance
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    z.linear.x = 0
    z.angular.z = 0
    #dist = 20
    resultant = 0
    # dist = math.sqrt((x-x_cor)**2+(y-y_cor)**2)
    angle = math.atan2((y_cor-y), (x_cor-x))-yaw
    g2g_linear = dist*(0.2)
    g2g_angular = angle*(0.4)
    if abs(g2g_angular) > 0.7:
        try:
            g2g_angular = abs(g2g_angular)*0.7/g2g_angular
        except:
            g2g_angular=0
    if g2g_linear > 0.7:
        g2g_linear = 0.7
    '''resultant_calculator'''
    max = 0
    for i in range(len(ranges)):
        if ranges[i] < 3.5:
            ranges[i] = 1-(ranges[i]/3.5)
        else:
            ranges[i] = 0
    for j in range(-60, 60):
        if (j >= 0):
            k = j
        else:
            k = 270+j
            j=(1.33)*j
        resultant -= (j)*ranges[k]
        if (max < ranges[k]):
            max = ranges[k]
    k_obs_linear = 3**(1-max)-1.5
    k_obs_angular = resultant/(700)
    k_g2g_linear=2.5-(math.pow(2.5,k_obs_linear))
    k_g2g_angular = (2.5-math.pow(2.5, k_obs_angular))
    final_linear = k_obs_linear + k_g2g_linear*(g2g_linear)
    final_angular = k_obs_angular + k_g2g_angular*g2g_angular
    
    # print(k_obs_angular,k_g2g_angular,g2g_angular)
    #print(f'angular_velocity={final_angular} and linear_velocity={final_linear}')
    z.linear.z=(2.9*final_angular+2*final_linear)*350
    z.linear.y=(2*final_linear-2.9*final_angular)*350
    pub.publish(z)
    rospy.sleep(0.4)

def odometryCb(msg):
    global x, y,distance
    x=msg.position.y
    y=msg.position.z
    distance=msg.position.x
def odometryCb2(msg):
    global yaw
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw=yaw*180/math.pi
    yaw=360-yaw
    if(yaw>360):
        yaw=yaw-360
    #print(yaw)


rospy.init_node('scan_node', anonymous=True)
rospy.Subscriber("scan", LaserScan, callback)
rospy.Subscriber('distance', Pose, odometryCb)
rospy.Subscriber('/android/imu',Imu,odometryCb2)
dist = 20
x_cor = float(input('Enter the x-coordinate of the goal:'))
y_cor = float(input('Enter the y-coordinate of the goal:'))
while not dist < 10 and not rospy.is_shutdown():
    calculator()
z.linear.x = 0
z.angular.z = 0
pub.publish(z)
rospy.spin()