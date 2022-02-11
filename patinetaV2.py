#!/usr/bin/env python3

from math import sqrt, atan2
import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose2D, PointStamped
import time 
import tf

class patineta:
    def __init__(self):
        self.rate = rospy.Rate(10) #hz
        self.elements = []
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
        self.distance_down = 0.15


        self.CommandArduino = False
        self.count = 0

        self.armPose = 0.0
        self.distanceToZone = 0.0
        self.zoneGoal = 0
        self.zone = 0
        self.okZoneGoal = False
        self.precisionArm = 30
        self.id = 0

        #self.pub = rospy.Publisher("time", Float32, queue_size=10)
        self.pubArduino = rospy.Publisher("cvTool_cmd", Bool, queue_size=10)
        self.pubZoneGoal = rospy.Publisher("cvArm_cmd", Int8, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.pose_callback)
        rospy.Subscriber('armPose', Float32, self.armPose_callback)
        rospy.Subscriber('weed_points', PointStamped, self.points_callback)

        rospy.loginfo("Run polar arm has been started")
        rospy.loginfo(self.elements)

        while not rospy.is_shutdown():
            #if (len(self.elements) > 0 or self.wait == True):
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
                            self.elements.pop(0)
                            print("El punto estaba demasiado cerca, ID:{}, puntos restantes: {} ".format(self.current_id,len(self.elements)))
                        #if (self.execute == False):
                        else:
                            # self.goal = self.element
                            print(self.zone)
                            self.pubZoneGoal.publish(int(self.zone))

                            #self.elements.pop(0)
                            self.execute = True
                            print("El punto paso a ejecucion, ID:{}, zona: {}".format(self.current_id, self.zone)) 
                    else:
                        self.elements.pop(0)
                        print("El punto quedo atras (THETA), ID:{}, puntos restantes: {} ".format(self.current_id,len(self.elements)))
                #-------------------------- execute function --------------------------
                else:
                    self.DistanceToLow = sqrt((self.point.x - self.pose.x) **2 + (self.point.y - self.pose.y) **2)
                    self.DistanceToZone = self.zoneGoal - self.armPose

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
                            self.elements.pop(0)
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
                            self.elements.pop(0)
                        print("Listorta, ID:{}, puntos restantes: {} ".format(self.current_id,len(self.elements)))
                    
                    if (abs(self.theta - self.pose.theta) > 1.57):
                        # print("goalZone: {}/ armPose: {} /distanceToZone: {}/ Nos pasamos segun angulo".format(self.zoneGoal, self.armPose, self.distanceToZone))
                        if(len(self.elements) > 0):
                            self.elements.pop(0)
                        print("Nos pasamos segun angulo, ID:{}, puntos restantes: {} ".format(self.current_id,len(self.elements)))
                        self.ok_goal = False
                        self.okZoneGoal = False
                        self.execute = False
                        self.wait = False
                     


                #else:
                #----------------------------------------------------------------------------

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
