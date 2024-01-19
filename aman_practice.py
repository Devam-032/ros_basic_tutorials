#!/usr/bin/env python3
import rospy,math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def odometryCb2(msg):
    print(1)
    yaw = 0
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw=yaw*180/math.pi
    yaw=360-yaw
    if(yaw>360):
        yaw=yaw-360
    print(yaw)
    
def listener():
    print(2)
    rospy.init_node('hmc_node', anonymous=True)
    print(4)
    rospy.Subscriber('android/imu',Imu,odometryCb2)
    rospy.spin()
if __name__ == '__main__':
    print(3)
    listener()