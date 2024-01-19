#!/usr/bin/env python3
import rospy, math
from sensor_msgs.msg import LaserScan,Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
pub = 0
def callback(data):
    global x, y, yaw, dist, x_cor, y_cor, ranges , pub
    ranges = [x for x in data.ranges]
    for i in range(len(ranges)):
            if (ranges[i]>3.5 or ranges[i]==0):
                ranges[i]=3.5
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
def calculations():
     pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
     global linear_velocity,angular_velocity
     global val
     val = Twist()
     pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
     min_dis1 = min(ranges[204:248])
     min_dis2 = min(min(ranges[248:270]),min(ranges[0:22])) 
     min_dis3 = min(ranges[22:66])
     if(min_dis1<0.5 or min_dis2<0.5 or min_dis3<0.5):
          if(min_dis1<0.5 and min_dis2<0.5):
               angular_velocity = math.pi/3
               linear_velocity = 50
          elif(min_dis2<0.5 and min_dis3<0.5):
               angular_velocity = -math.pi/3
               linear_velocity = 50
          else:
               angular_velocity = 1.5*(min_dis3-min_dis1)
               linear_velocity = 80
     else:
          angular_velocity = 1.5*(yaw - angle_1)
          linear_velocity = distance
     val.linear.x = linear_velocity
     val.angular.z = angular_velocity
     pub.publish(val)
rospy.init_node('scan_node', anonymous=True)
rospy.Subscriber("scan", LaserScan, callback)
rospy.Subscriber('distance', Pose, odometryCb)
rospy.Subscriber('/android/imu',Imu,odometryCb2)
x_cor = float(input('Enter the x-coordinate of the goal:'))
y_cor = float(input('Enter the y-coordinate of the goal:'))
# x_cor=21.1658157
# y_cor=72.78512222 
dist = 20
while not dist < 10 and not rospy.is_shutdown():
    calculations()
val.linear.x = 0
val.angular.z = 0
pub.publish(val)
rospy.spin()                     
               
               