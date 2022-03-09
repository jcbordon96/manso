#!/usr/bin/env python3

from math import sqrt, atan2
import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Float32, Bool, String
from geometry_msgs.msg import Point, Pose2D, PointStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix
import csv
import time 
import tf
import os
from datetime import datetime

class patineta:
    def __init__(self):
        self.rate = rospy.Rate(10) #hz
        self.elements = []
        self.ok_elements = []
        self.failed_elements = []
        
        self.point = 0.0
        self.time = 0.0
        self.goal = 0.0
        self.distance = 0.0
        self.previous_goal = 0.0
        self.firstPoint = True
        self.new_goal = True
        self.wait = False

        self.TimeConstant = 1.0
        self.vel_x = 0.5
        self.HoldStop = False
        self.timeBetweenSquares = 0.0
        
        self.execute = False
        self.DistanceToLow = 0.0
        self.DistanceDown = 0.0
        self.point = Point()
        self.pose = Pose2D()
        self.ok_goal = False
        self.stop_flag = False
        self.distance_to_low = 0.05
        self.distance_down = 0.1


        self.CommandArduino = False
        self.count = 0

        self.armPose = 0.0
        self.distanceToZone = 0.0
        self.zoneGoal = 0
        self.zone = 0
        self.okZoneGoal = False
        self.precisionArm = 30
        self.id = 0
        self.tool_status = True
        self.success_rate = Float32()
        self.gnss = NavSatFix()

        #self.pub = rospy.Publisher("time", Float32, queue_size=10)
        self.pubArduino = rospy.Publisher("cvTool_cmd", Bool, queue_size=10)
        self.pubZoneGoal = rospy.Publisher("cvArm_cmd", Int8, queue_size=10)
        self.pub_success_rate = rospy.Publisher("arm_success_rate", Float32, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.pose_callback)
        rospy.Subscriber('armPose', Float32, self.armPose_callback)
        rospy.Subscriber('weed_points', PointStamped, self.points_callback)
        rospy.Subscriber('tool_status', Bool, self.tool_status_callback)
        rospy.Subscriber('gnss', NavSatFix, self.gnss_callback)
        rospy.Subscriber('gnss_time', String, self.gnss_time_callback)
        

        rospy.loginfo("Run polar arm has been started")
        rospy.loginfo(self.elements)

        while not rospy.is_shutdown():
            #if (len(self.elements) > 0 or self.wait == True):
            if self.wait_gnss:
                if (len(self.elements) > 0):
                    #if (self.wait == False):
                    self.point.x = self.elements[0][0]
                    self.point.y = self.elements[0][1]
                    self.zone = self.elements[0][2]
                    self.current_id = self.elements[0][3]
                    # if (self.zone == 1):
                    #     self.zoneGoal = 25
                    # elif (self.zone == 2):
                    #     self.zoneGoal = 89
                    # elif (self.zone == 3):
                    #     self.zoneGoal = 151
                    self.distance = sqrt((self.point.x - self.pose.x) **2 + (self.point.y - self.pose.y) **2)
                    self.theta = atan2((self.point.y - self.pose.y), (self.point.x - self.pose.x))
                    if  not self.execute:
                        if (abs(self.theta - self.pose.theta) < 1.57):
                            print(self.distance)
                            if (self.distance < self.distance_down):
                                self.failed_elements.append(self.elements.pop(0))
                                self.logwriter(self.gnss.latitude, self.gnss.longitude, 0, self.current_id)
                                print("El punto estaba demasiado cerca, ID:{}, puntos restantes: {} ".format(self.current_id,len(self.elements)))
                            #if (self.execute == False):
                            else:
                                # self.goal = self.element
                                while not self.tool_status:
                                    time.sleep(0.1)
                                self.tool_status = False
                                print(self.zone)
                                self.pubZoneGoal.publish(int(self.zone))

                                #self.elements.pop(0)
                                self.execute = True
                                print("El punto paso a ejecucion, ID:{}, zona: {}".format(self.current_id, self.zone)) 
                        else:
                            self.failed_elements.append(self.elements.pop(0))
                            self.logwriter(self.gnss.latitude, self.gnss.longitude, 0, self.current_id)
                            print("El punto quedo atras (THETA), ID:{}, puntos restantes: {} ".format(self.current_id,len(self.elements)))
                    #-------------------------- execute function --------------------------
                    else:
                        
                        self.DistanceToLow = sqrt((self.point.x - self.pose.x) **2 + (self.point.y - self.pose.y) **2)
                        self.DistanceToZone = self.zoneGoal - self.armPose
                        print("point: {}, pose: {}, distance: {}".format(self.point.x, self.pose.x, self.DistanceToLow))
                        # if (self.okZoneGoal == False or self.ok_goal == False):
                            # print("goalX: {}/ mansoPose: {} /distanceToStart: {} /goalZone: {}/ armPose: {} /distanceToZone: {}".format(self.goal, self.encoder, self.DistanceToLow, self.zoneGoal, self.armPose, self.distanceToZone))

                            #print("goalZone: {}/ armPose: {} /distanceToZone: {}".format(self.zoneGoal, self.armPose, self.distanceToZone))
                        if (self.DistanceToZone <= self.precisionArm and not self.okZoneGoal):
                            self.okZoneGoal = True
                            print("El brazo esta en posicion: {}, ID: {}".format(self.zone, self.current_id))
                        
                        if (self.DistanceToLow <= self.distance_to_low and not self.ok_goal):
                            self.ok_goal = True
                            print("El brazo esta listo para accionar, ID: ", self.current_id)
                        
                        if (self.okZoneGoal == False and self.ok_goal == True):
                            # print("goalZone: {}/ armPose: {} /distanceToZone: {}/ No se llego a la zona a tiempo".format(self.zoneGoal, self.armPose, self.distanceToZone))
                            if(len(self.elements) > 0):
                                self.failed_elements.append(self.elements.pop(0))
                                self.logwriter(self.gnss.latitude, self.gnss.longitude, 0, self.current_id)
                            print("El brazo no llego a su posicion antes del objetivo, ID:{}, puntos restantes: {} ".format(self.current_id,len(self.elements)))
                            self.ok_goal = False
                            self.okZoneGoal = False
                            self.execute = False
                            self.wait = False
                            

                        if (self.okZoneGoal == True and self.ok_goal == True and self.stop_flag == False):
                            
                            print("Voy, ID: ", self.current_id)
                            self.time = time.perf_counter()
                            self.CommandArduino = True
                            self.pubArduino.publish(self.CommandArduino)
                            # print ("patineta ON")
                            self.ok_goal = False
                            self.okZoneGoal = False
                            self.execute = False
                            self.wait = False
                            if(self.CommandArduino == True):
                                self.CommandArduino = False
                            if(len(self.elements) > 0):
                                self.ok_elements.append(self.elements.pop(0))
                                self.logwriter(self.gnss.latitude, self.gnss.longitude, 1, self.current_id)
                            print("Listorta, ID:{}, puntos restantes: {} ".format(self.current_id,len(self.elements)))
                        
                        if (abs(self.theta - self.pose.theta) > 1.57):
                            # print("goalZone: {}/ armPose: {} /distanceToZone: {}/ Nos pasamos segun angulo".format(self.zoneGoal, self.armPose, self.distanceToZone))
                            if(len(self.elements) > 0):
                                self.failed_elements.append(self.elements.pop(0))
                                self.logwriter(self.gnss.latitude, self.gnss.longitude, 0, self.current_id)
                            print("Nos pasamos segun angulo, ID:{}, puntos restantes: {} ".format(self.current_id,len(self.elements)))
                            self.ok_goal = False
                            self.okZoneGoal = False
                            self.execute = False
                            self.wait = False
                        if (len(self.ok_elements)+len(self.failed_elements)) > 0:
                            self.success_rate = round(len(self.ok_elements)/(len(self.ok_elements)+len(self.failed_elements)),2)
                        else:
                            self.success_rate = 100
                        self.pub_success_rate.publish(self.success_rate)
                        


                #else:
                #----------------------------------------------------------------------------
    def logwriter(self, lat, lon, state, id):
        nowlogdate = datetime.now()
        logdate = nowlogdate.strftime("%Y-%m-%d")
        loghour = nowlogdate.strftime("%H:%M:%S")
        datename = nowlogdate.strftime("%Y%m%d%H%M%S")
        stringdatelog = "/home/appelie/manso_ws/src/v1/scripts/manso/resources/logs/success/diario/"+datename+".csv"
        stringdatelogbackup = "/home/appelie/manso_ws/src/v1/scripts/manso/resources/logs/success/backup/"+datename+".csv"

        if not os.path.exists(stringdatelog):
            print("No existe el logfile diario")
            header = [ "Fecha", "Hora", "Latitud", "Longitud", "Estado", "ID"]
            with open(stringdatelog, 'w') as logfile:
                wr = csv.writer(logfile)
                wr.writerow(header)
        with open(stringdatelog, 'a', newline='') as logfile:
            wr = csv.writer(logfile)
            wr.writerow([logdate, loghour, lat, lon, state, id])

        if not os.path.exists(stringdatelogbackup):
            print("No existe el logfile diario de backup")
            header = [ "Fecha", "Hora", "Latitud", "Longitud", "Estado", "ID"]
            with open(stringdatelogbackup, 'w') as logfile:
                wr = csv.writer(logfile)
                wr.writerow(header)
        with open(stringdatelogbackup, 'a', newline='') as logfile:
            wr = csv.writer(logfile)
            wr.writerow([logdate, loghour, lat, lon, state, id])

        if not os.path.exists('/home/appelie/manso_ws/src/v1/scripts/manso/resources/logs/success/success.csv'):
            print("No existe el logfile diario de backup")
            header = [ "Fecha", "Hora", "Latitud", "Longitud", "Estado", "ID"]
            with open('/home/appelie/manso_ws/src/v1/scripts/manso/resources/logs/success/success.csv', 'w') as logfile:
                wr = csv.writer(logfile)
                wr.writerow(header)

        with open('/home/appelie/manso_ws/src/v1/scripts/manso/resources/logs/success/success.csv', 'a', newline='') as logfile:
            wr = csv.writer(logfile)
            wr.writerow([logdate, loghour, lat, lon, state, id])

    def tool_status_callback(self, msg):
        self.tool_status = msg.data
    def gnss_time_callback(self, msg):
        self.time = msg.data

    def gnss_callback(self, msg):
        self.gnss = msg
        self.wait_gnss = True
        
    def pose_callback(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.pose.theta = y
        
    def armPose_callback(self, msg):
        self.armPose = msg.data 
    def points_callback(self, msg):
        self.elements.append([msg.point.x, msg.point.y, msg.point.z, self.id])
        print("Nuevo punto con el ID: ", self.id)
        self.id += 1
        

if __name__ == '__main__':
    try:
        rospy.init_node('patin_control', anonymous=True)
        patineta = patineta()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
