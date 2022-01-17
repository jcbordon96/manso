#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg  import Odometry
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3, Pose
v_x = 0
v_y = 0
vth = 0


def callback(msg):
    global v_x
    global v_y
    v_x = (msg.linear.x + msg.linear.y)/2
    v_y = (msg.linear.y + msg.angular.x)/2


rospy.init_node('odometry')
rospy.Subscriber('wheel_encoders', Twist, callback)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()
x = 0.0
y = 0.0
th = 0.0
vx = (v_x+v_y)/2
vy = 0.0
vth = (v_x-v_y)/1.3
current_time = rospy.Time.now()
last_time = rospy.Time.now()
r = rospy.Rate(10.0)
while not rospy.is_shutdown():
    vx = (v_x+v_y)/2
    vy = 0.0
    vth = (v_x-v_y)/1.3

    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th))*dt
    delta_y = (vx * sin(th))*dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    odom_quat = tf.transformations.quaternion_from_euler(0,0,th)

    odom_broadcaster.sendTransform((x,y,0.),odom_quat,current_time,"base_link","odom")

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x,y,0.),Quaternion(*odom_quat))

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx,vy,0), Vector3(0,0,vth))

    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
