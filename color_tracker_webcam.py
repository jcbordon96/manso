#!/usr/bin/env python3

from dis import dis
from turtle import distance
import rospy
import cv2
import color_tracker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import tf
import math


class WeedTracker:
    pose = Point()
    point = Point()
    last_pose = Point()
    global objects
    objects = -1
    close = False
    distance_to_take_pic = 0.35
    weed = "X"
    gnss = NavSatFix()
    time = ""
    take_pic = True
    distance = 0
    def __init__(self):
        retry = True
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.Subscriber('gnss', NavSatFix, self.gnss_callback)
        rospy.Subscriber('gnss_time', String, self.gnss_callback)

        self.pub_points = rospy.Publisher("weed_points", Point, queue_size=10)
        rospy.loginfo("Iniciado")
        tracker = color_tracker.ColorTracker(max_nb_of_objects=15, max_nb_of_points=20, debug=True)
        tracker.set_tracking_callback(self.tracker_callback)
        while True:
            if retry:
                try:
                    with color_tracker.WebCamera(video_src="/dev/TRACKER") as cam:
                        # Define your custom Lower and Upper HSV values
                        retry = False
                        tracker.track(cam, [30,20,20], [90,255,255], max_skipped_frames=30, min_contour_area=500) 
                except Exception as e:
                    print("Algo salio mal, error: ", e)
                    print(self.close)
                    if self.close == False:
                        retry = True
                    else:
                        break

    def odom_callback(self, msg):
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        self.point.x = msg.pose.pose.position.x + 0.88 * math.cos(y)
        self.point.y = msg.pose.pose.position.y + 0.88 * math.sin(y)
        self.pose.x = msg.pose.pose.position.x 
        self.pose.y = msg.pose.pose.position.y 
    
    def gnss_callback(self, msg):
        self.gnss = msg

    def gnss_callback(self, msg):
        self.time = msg.data

    def updateObjects(self):
        global objects
        objects = 1


    def tracker_callback(self, t: color_tracker.ColorTracker):
        global objects
        if len(t.tracked_objects) > 0:
            self.weed = "Y"
        else:
            self.weed = "N"
        # print("There is a plant :", self.weed)
        if self.take_pic == True:
            string_clear = "resources/clear/" + self.time + "_" + str(self.gnss.latitude) + "_" + str(self.gnss.longitude) + "_C_" + self.weed + ".png"
            print(string_clear)
            print(cv2.imwrite(string_clear, t.frame))
            string_debug = "resources/debug/" + self.time + "_" + str(self.gnss.latitude) + "_" + str(self.gnss.longitude) + "_D_" + self.weed + ".png"
            print(string_debug)
            print(cv2.imwrite(string_debug, t.debug_frame))
            self.last_pose = self.pose
            self.take_pic = False
        print((self.pose.x - self.last_pose.x))
        if self.distance != math.sqrt((self.pose.x - self.last_pose.x) **2 + (self.pose.y - self.last_pose.y) **2):
            self.distance = math.sqrt((self.pose.x - self.last_pose.x) **2 + (self.pose.y - self.last_pose.y) **2)
        if self.distance > self.distance_to_take_pic:
            self.take_pic = True
            print("Voy a sacar una foto")

        for i in range(len(t.tracked_objects)):
            
            if(objects < t.tracked_objects[i]._id):
                print("Punto")
                objects = t.tracked_objects[i]._id
                # pointX = (t.tracked_objects[i].last_point[0]*640) /175
                pointX = t.tracked_objects[i].last_point[0]
                # print(pointX)
                # print("pixel:" + str (t.tracked_objects[i].last_point[0]))
                
                if(pointX > 150 and pointX < 263 ):
                    print("1")
                    self.point.z = 1
                elif(pointX > 263 and pointX < 375 ):
                    print("2")
                    self.point.z = 2
                elif(pointX > 375 and pointX < 490 ):
                    print("3")
                    self.point.z = 3
                self.pub_points.publish(self.point)
            #cv2.circle(t.debug_frame, t.tracked_objects[i].last_point, 5, [255,255,255], 3)  
        
        res = cv2.resize(t.debug_frame, (640,480))
        cv2.rectangle(res, (150,0), (150,480),[255,0,255], 3)
        cv2.rectangle(res, (490,0), (490,480),[255,0,255], 3)
        cv2.rectangle(res, (263,0), (263,480),[0,255,0], 3)
        cv2.rectangle(res, (0,240), (640,240),[0,255,0], 3)
        cv2.rectangle(res, (375,0), (375,480),[0,255,0], 3)
        cv2.imshow("debug", res)
        # cv2.waitKey(1)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            self.close = True
            raise Exception("Close")
            # break


if __name__ == "__main__":
    rospy.init_node("weed_tracker")
    try:
        weed_tracker = WeedTracker()
        rospy.spin()
    except KeyboardInterrupt:
        print("Chau")
    
