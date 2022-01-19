#!/usr/bin/env python3

import rospy
import cv2
import color_tracker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import tf
import math


class WeedTracker:
    pose = Point()
    global objects
    objects = -1
    close = False
    def __init__(self):
        retry = True
        rospy.Subscriber('odom', Odometry, self.odom_callback)
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

        self.pose.x = msg.pose.pose.position.x + 0.88 * math.cos(y)
        self.pose.y = msg.pose.pose.position.x + 0.88 * math.sin(y)

    def updateObjects(self):
        global objects
        objects = 1


    def tracker_callback(self, t: color_tracker.ColorTracker):
        global objects
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
                    self.pose.z = 1
                elif(pointX > 263 and pointX < 375 ):
                    print("2")
                    self.pose.z = 2
                elif(pointX > 375 and pointX < 490 ):
                    print("3")
                    self.pose.z = 3
                self.pub_points.publish(self.pose)
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
    
