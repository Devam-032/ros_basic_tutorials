#!/usr/bin/python3
import time
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

distance, distance_current , sub , sub1, x_init , y_init , x1 , y1 = 0, 0, 0, 0, 0, 0 , 0 , 0


def poseCallback():

    global  distance_current,sub ,distance,x_init,y_init,x1,y1

    x = x_init
    y = y_init

    # print(f'x_init:{x_init}')
    # print(f'x:{x}')
    # print(f'y:{y}')
    # print(f'x1:{x1}')
    # print(f'y1:{y1}')
    # theta = pose.theta
    distance_current = x -x1
    # print("distance:")
    # print(distance_current,distance)

    linear_velocity = (distance - distance_current)/3
    angular_velocity = 0
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    z = Twist()
    z.linear.x = linear_velocity
    z.angular.z = angular_velocity
    pub.publish(z)

def poseCallback1(pose):
    global x_init , y_init , distance , distance_current
    x_init = pose.x
    y_init = pose.y
    print(x_init,y_init)
    main()

def main():
    global distance,  sub , sub1 , x_init , y_init, distance_current , x1 , y1
    print(x1,y1,distance_current-distance)
    if(abs(distance_current-distance)<=0.2):
        distance = float(input('Enter Distance'))
        x1 = x_init
        y1 = y_init

        

    elif(abs(distance_current-distance)>0.2):
        poseCallback()



if __name__ == "__main__":
    rospy.init_node('turtlesim_motion_pose')
    while (x_init==0 and y_init==0):
        sub1 = rospy.Subscriber('/turtle1/pose', Pose, poseCallback1)
    sub1 = rospy.Subscriber('/turtle1/pose', Pose, poseCallback1)
    rospy.spin()