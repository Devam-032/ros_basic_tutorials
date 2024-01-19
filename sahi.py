#!/usr/bin/python3
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

theta,theta1,sub , a = 0,0,0,0

# a = 0 jab arrow detect ho and align na hogya ho
# a = 1 jab align hogya ho
        

def callback(data):
    global  theta , a , theta1 , sub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    y1 , y2 , center=min_range_index(data.ranges)
    z = Twist()
    print(y1-y2,center)
    
    if(a == 0):
        angular_velocity= 0
        linear_velocity = .1
        if(center>3.0):
            #timer
            angular_velocity = y1-y2
            linear_velocity = 0
            if(abs(y1-y2)<0.01 ):
                linear_velocity = 0
                angular_velocity = 0
                z.linear.x = linear_velocity
                z.angular.z = angular_velocity
                a =1 
                theta1 = theta + math.pi/2
                pub.publish(z)

        z.linear.x = linear_velocity
        z.angular.z = angular_velocity
        pub.publish(z)
    if(a==1):
        angular_velocity = (theta1 - theta )
        linear_velocity = 0
        if(angular_velocity<0.1):
            angular_velocity= 0
            a=0
        z.linear.x = linear_velocity
        z.angular.z = angular_velocity
        pub.publish(z)
        

        

    #print(leny)
    

def min_range_index(ranges):
    rangess = [x for x in ranges]
    for i in range(len(rangess)):
        if (math.isnan(rangess[i])):
            rangess[i]=3.5
        elif (rangess[i]>3.5):
            rangess[i]=3.5
    #print(rangess , len(rangess))
    b = 3.5
    y1 = [(b - rangess[i])*i for i in range (1 , 5)]
    y2 = [(b-rangess[len(rangess)-i])*i for i in range (1 , 5)] 
    center = b -rangess[0]
    y11 = sum(y1)
    y22 = sum(y2)
    #print(y,len(y))
    return(y11, y22 , center )

def poseCallback1(msg):
    global theta
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, theta) = euler_from_quaternion(orientation_list)

def main():
    global  sub, theta
    sub = rospy.Subscriber('odom', Odometry, poseCallback1)

if __name__ == '__main__':
    rospy.init_node('scan_node', anonymous=True)
    main()
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()