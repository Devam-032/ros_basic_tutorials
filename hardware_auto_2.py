#!/usr/bin/env python3
import rospy, math
from sensor_msgs.msg import LaserScan,Imu
from geometry_msgs.msg import Twist,Pose
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler
x, y,alpha1,angle_1,distance= 0, 0,math.pi/2,0,0
def callback(data):
    global x, y, yaw, dist, x_cor, y_cor, ranges
    ranges = [x for x in data.ranges]
    for i in range(len(ranges)):
            if (ranges[i]>1.5 or ranges[i]==0):
                ranges[i]=1.5

def calculator():
    global dist, x_cor, y_cor, z, pub,ranges,distance,yaw,angle_1
    z = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    #dist = 20
    resultant = 0
    dist = distance
    angle_1 = angle_1*math.pi/180
    angle = -math.atan2((y_cor-y), (x_cor-x))-yaw
    # if(angle<-math.pi):
    #     angle = angle+2*math.pi
    # if(angle>math.pi):
    #     angle = angle - 2*math.pi
    
    if (abs(angle) > 3.14):
        angle = angle-6.28*(abs(angle)/angle)
    z.angular.x = angle 
    print(yaw*180/math.pi,-math.atan2((y_cor-y), (x_cor-x))*180/math.pi,angle*180/math.pi)
    g2g_linear = dist*(0.2)
    g2g_angular = angle*(0.4)
    
    if abs(g2g_angular) > 0.3:
        g2g_angular = 0.3*abs(g2g_angular)/g2g_angular
    if g2g_linear > 0.3:
        g2g_linear = 0.3*abs(g2g_linear)/g2g_linear
    '''resultant_calculator'''
    max = 0
    for i in range(len(ranges)):
        if ranges[i] < 1.5:
            ranges[i] = 1-(ranges[i]/1.5)
        else:
            ranges[i] = 0
    for j in range(-45, 45):
        if (j >= 0):
            k = j
        else:
            k = len(ranges)+j
        resultant -= (j)*ranges[k]
        if (max < ranges[k]):
            max = ranges[k]
    print(max)
    k_obs_linear = 3**(1-max)-1.5
    print(k_obs_linear)
    k_obs_angular = resultant/(700)
    #print(k_obs_angular)
    k_g2g_linear=2.5-(math.pow(2.5,abs(k_obs_linear)))
    k_g2g_angular = (2.5-math.pow(2.5, abs(k_obs_angular)))
    z.linear.x = k_obs_linear + k_g2g_linear*(g2g_linear)
    # print(z.linear.x)
    if (z.linear.x > 0.1):
        z.linear.x = 0.1
    z.angular.z = 1.8*k_obs_angular+ (0.6*k_g2g_angular*g2g_angular)
    #print(z.angular.z)
    #print(f'k_obs_angular:{k_obs_angular} k_g2g_angular:{k_g2g_angular} g2g_angular:{g2g_angular} angular={z.angular.z}')

    if (abs(z.angular.z) > 0.4):
        z.angular.z = 0.4*abs(z.angular.z)/(z.angular.z)
    
    z.linear.z = 0
    z.linear.y= 0
    z.linear.z=(1800*z.linear.x+750*z.angular.z)
    z.linear.y=(1800*z.linear.x-750*z.angular.z)
    # z.angular.x = angle
    z.angular.y=yaw
    #print(z.angular.z)
    pub.publish(z)
    #rospy.loginfo(z)
    rospy.sleep(0.4)

def odometryCb(msg):
    global x, y,distance,angle_1
    x=msg.position.y
    y=msg.position.z
    distance=msg.position.x
    angle_1=msg.orientation.y
def odometryCb2(msg):
    global yaw
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw=yaw
    
    

    # if(yaw>6.28):
    #     yaw=yaw-6.28
    #print(yaw)

rospy.init_node('scan_node', anonymous=True)
rospy.Subscriber("scan", LaserScan, callback)
rospy.Subscriber('distance', Pose, odometryCb)
rospy.Subscriber('/arimu',Float64,odometryCb2)
x_cor = float(input('Enter the x-coordinate of the goal:'))
y_cor = float(input('Enter the y-coordinate of the goal:'))
# x_cor=21.1658157
# y_cor=72.78512222 
dist = 20
while not dist < 10 and not rospy.is_shutdown():
    calculator()
z.linear.x = 0
z.angular.z = 0
pub.publish(z)
rospy.spin()