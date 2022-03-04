#!/usr/bin/env python3

from multiprocessing.connection import wait
import rospy
import cv2
import color_tracker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import tf
import math
import os
from datetime import datetime
import csv
import time as ptime

class WeedTracker:
    pose = Point()
    point = PointStamped()
    last_pose = Point()
    point.header.frame_id = "odom"
    distance_lowerCam_tool = 0.85
    px_to_cm = 1200
    pointY_odom = 0
    global objects
    objects = -1
    close = False
    distance_to_take_pic = 0.35
    weed = "X"
    gnss = NavSatFix()
    time = ""
    take_pic = True
    distance = 0
    wait_gnss = False
    fileTime=int(ptime.time())
    os.mkdir('/home/appelie/manso_ws/src/v1/scripts/manso/resources/videos/tracker/{}'.format(fileTime))
    out_debug = cv2.VideoWriter('/home/appelie/manso_ws/src/v1/scripts/manso/resources/videos/tracker/{}/debug.avi'.format(fileTime), cv2.VideoWriter_fourcc(*'MJPG'),10, (640,480))
    out_raw = cv2.VideoWriter('/home/appelie/manso_ws/src/v1/scripts/manso/resources/videos/tracker/{}/raw.avi'.format(fileTime), cv2.VideoWriter_fourcc(*'MJPG'),10, (640,480))

    def __init__(self):
        retry = True
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.Subscriber('gnss', NavSatFix, self.gnss_callback)
        rospy.Subscriber('gnss_time', String, self.gnss_time_callback)

        self.pub_points = rospy.Publisher("weed_points", PointStamped, queue_size=10)
        rospy.loginfo("Iniciado")
        tracker = color_tracker.ColorTracker(max_nb_of_objects=15, max_nb_of_points=5000, debug=True)
        tracker.set_tracking_callback(self.tracker_callback)
        
        while True:
            if retry:
                try:

                    with color_tracker.WebCamera(video_src="/dev/TRACKER") as cam:
                        # Define your custom Lower and Upper HSV values
                        retry = False
                        #tracker.track(cam, [30,20,20], [90,255,255], max_skipped_frames=30, min_contour_area=500) #green
                        tracker.track(cam, [136,87,111], [180,255,255], max_skipped_frames=30, min_contour_area=500) #red
                except Exception as e:
                    print("Algo salio mal, error: ", e)
                    print(self.close)
                    if self.close == False:
                        retry = True
                    else:
                        break

    def odom_callback(self, msg):
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.point.point.x = msg.pose.pose.position.x + 1.2 * math.cos(y) - (self.pointY_odom / self.px_to_cm) * math.cos(y) 
        self.point.point.y = msg.pose.pose.position.y + 1.2 * math.sin(y) - (self.pointY_odom / self.px_to_cm) * math.sin(y)
        self.pose.x = msg.pose.pose.position.x 
        self.pose.y = msg.pose.pose.position.y 
    
    def gnss_callback(self, msg):
        self.gnss = msg
        self.wait_gnss = True

    def gnss_time_callback(self, msg):
        self.time = msg.data

    def updateObjects(self):
        global objects
        objects = 1

    def logwriter(self, lat, lon, state):
        nowlogdate = datetime.now()
        logdate = nowlogdate.strftime("%Y-%m-%d")
        loghour = nowlogdate.strftime("%H:%M:%S")
        datename = nowlogdate.strftime("%Y%m%d")
        stringdatelog = "/home/appelie/manso_ws/src/v1/scripts/manso/resources/logs/maps/diario/"+datename+".csv"
        stringdatelogbackup = "/home/appelie/manso_ws/src/v1/scripts/manso/resources/logs/maps/backup/"+datename+".csv"

        if not os.path.exists(stringdatelog):
            print("No existe el logfile diario")
            header = [ "Fecha", "Hora", "Latitud", "Longitud", "Estado"]
            with open(stringdatelog, 'w') as logfile:
                wr = csv.writer(logfile)
                wr.writerow(header)
        with open(stringdatelog, 'a', newline='') as logfile:
            wr = csv.writer(logfile)
            wr.writerow([logdate, loghour, lat, lon, state])

        if not os.path.exists(stringdatelogbackup):
            print("No existe el logfile diario de backup")
            header = [ "Fecha", "Hora", "Latitud", "Longitud", "Estado"]
            with open(stringdatelogbackup, 'w') as logfile:
                wr = csv.writer(logfile)
                wr.writerow(header)
        with open(stringdatelogbackup, 'a', newline='') as logfile:
            wr = csv.writer(logfile)
            wr.writerow([logdate, loghour, lat, lon, state])

        if not os.path.exists('/home/appelie/manso_ws/src/v1/scripts/manso/resources/logs/maps/map.csv'):
            print("No existe el logfile diario de backup")
            header = [ "Fecha", "Hora", "Latitud", "Longitud", "Estado"]
            with open('/home/appelie/manso_ws/src/v1/scripts/manso/resources/logs/maps/map.csv', 'w') as logfile:
                wr = csv.writer(logfile)
                wr.writerow(header)

        with open('/home/appelie/manso_ws/src/v1/scripts/manso/resources/logs/maps/map.csv', 'a', newline='') as logfile:
            wr = csv.writer(logfile)
            wr.writerow([logdate, loghour, lat, lon, state])

    def tracker_callback(self, t: color_tracker.ColorTracker):
        self.out_debug.write(t.debug_frame)
        self.out_raw.write(t.frame)
        if self.wait_gnss:
            global objects
            if len(t.tracked_objects) > 0:
                self.weed = "Y"
            else:
                self.weed = "N"
            # print("There is a plant :", self.weed)
            self.distance = math.sqrt((self.pose.x - self.last_pose.x) **2 + (self.pose.y - self.last_pose.y) **2)
            # print(self.distance)
            if self.distance > self.distance_to_take_pic:
                self.take_pic = True
                print("Voy a sacar una foto")
            if self.take_pic == True:
                string_clear = "/home/appelie/manso_ws/src/v1/scripts/manso/resources/images/clear/" + self.time + "_" + str(self.gnss.latitude) + "_" + str(self.gnss.longitude) + "_C_" + self.weed + ".png"
                print(string_clear)
                print(cv2.imwrite(string_clear, t.frame))
                string_debug = "/home/appelie/manso_ws/src/v1/scripts/manso/resources/images/debug/" + self.time + "_" + str(self.gnss.latitude) + "_" + str(self.gnss.longitude) + "_D_" + self.weed + ".png"
                print(string_debug)
                print(cv2.imwrite(string_debug, t.debug_frame))
                self.logwriter(self.gnss.latitude, self.gnss.longitude, self.weed)
                self.last_pose.x = self.pose.x
                self.last_pose.y = self.pose.y
                self.take_pic = False
        

        for i in range(len(t.tracked_objects)):
            
            if(objects < t.tracked_objects[i]._id):
                #print("Punto Nuevo")
                #objects = t.tracked_objects[i]._id
                # pointX = (t.tracked_objects[i].last_point[0]*640) /175
                pointX = t.tracked_objects[i].last_point[0]
                pointY = t.tracked_objects[i].last_point[1]
                self.pointY_odom = pointY
                # print(pointX)
                # print("pixel:" + str (t.tracked_objects[i].last_point[0]))
                if (pointY < 240):
                    print("Punto Nuevo")
                    print(pointY)
                    objects = t.tracked_objects[i]._id    
                    if (pointX > 150 and pointX < 490):
                        if(pointX > 150 and pointX < 263 ):
                            print("1")
                            self.point.point.z = 1
                        elif(pointX > 263 and pointX < 375 ):
                            print("2")
                            self.point.point.z = 2
                        elif(pointX > 375 and pointX < 490 ):
                            print("3")
                            self.point.point.z = 3
                        self.pub_points.publish(self.point)
                # print(self.point)
                
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
    
