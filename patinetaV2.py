#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import time 

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
        self.encoder = 0.0 
        self.ok_goal = False
        self.stop_flag = False
        self.distance_to_low = 0.05
        self.distance_down = 0.15
        self.rate = rospy.Rate(1)

        self.CommandArduino = False
        self.count = 0

        self.armPose = 0.0
        self.distanceToZone = 0.0
        self.zoneGoal = 0
        self.zone = 0
        self.okZoneGoal = False
        self.precisionArm = 15

        #self.pub = rospy.Publisher("time", Float32, queue_size=10)
        self.pubArduino = rospy.Publisher("cvTool_cmd", Bool, queue_size=10)
        self.pubZoneGoal = rospy.Publisher("cvArm_cmd", Int8, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.pose_callback)
        rospy.Subscriber('armPose', Float32, self.armPose_callback)
        rospy.Subscriber('weed_points', Point, self.points_callback)

        rospy.loginfo("Run polar arm has been started")
        rospy.loginfo(self.elements)

        while not rospy.is_shutdown():
            #if (len(self.elements) > 0 or self.wait == True):
            if (len(self.elements) > 0):
                #if (self.wait == False):
                self.element = self.elements[0][0]
                self.zone = self.elements[0][1]
                if (self.zone == 1):
                    self.zoneGoal = 25
                if (self.zone == 2):
                    self.zoneGoal = 89
                if (self.zone == 3):
                    self.zoneGoal = 151
                if (self.element > self.goal):
                    self.distance = self.element - self.goal
                    print(self.element)
                    if (self.distance < self.distance_down):
                        self.elements.pop(0)
                        print("punto filtrado")
                    #if (self.execute == False):
                    if (self.distance > self.distance_down):
                        self.goal = self.element
                        self.pubZoneGoal.publish(self.zone)
                        print (self.zone)
                        #self.elements.pop(0)
                        self.execute = True
                        print("pase el punto a goal") 
                #-------------------------- execute function --------------------------
                if (self.execute == True):
                    self.DistanceToLow = self.goal - self.encoder
                    self.DistanceToZone = self.zoneGoal - self.armPose

                    if (self.okZoneGoal == False or self.ok_goal == False):
                        print("goalX: {}/ mansoPose: {} /distanceToStart: {} /goalZone: {}/ armPose: {} /distanceToZone: {}".format(self.goal, self.encoder, self.DistanceToLow, self.zoneGoal, self.armPose, self.distanceToZone))

                        #print("goalZone: {}/ armPose: {} /distanceToZone: {}".format(self.zoneGoal, self.armPose, self.distanceToZone))
                        if (self.DistanceToZone <= self.precisionArm):
                            self.okZoneGoal = True
                            print("arm position achieved")
                        
                        if (self.DistanceToLow <= self.distance_to_low):
                            self.ok_goal = True
                            print("goal position achieved")
                    
                    if (self.okZoneGoal == False and self.ok_goal == True):
                        print("goalZone: {}/ armPose: {} /distanceToZone: {}/ No se llego a la zona a tiempo".format(self.zoneGoal, self.armPose, self.distanceToZone))
                        self.ok_goal = False
                        self.okZoneGoal = False
                        self.execute = False
                        self.wait = False
                        if(len(self.elements) > 0):
                            self.elements.pop(0)

                    if (self.okZoneGoal == True and self.ok_goal == True and self.stop_flag == False):
                        print("ready to START")
                        self.time = time.clock()
                        self.CommandArduino = True
                        self.pubArduino.publish(self.CommandArduino)
                        print ("patineta ON")
                        self.ok_goal = False
                        self.okZoneGoal = False
                        self.execute = False
                        self.wait = False
                        if(self.CommandArduino == True):
                            self.CommandArduino = False
                        if(len(self.elements) > 0):
                            self.elements.pop(0)
                        print("end process")

                #else:
                #----------------------------------------------------------------------------
 
    def pose_callback(self, msg):
        self.encoder = msg.pose.pose.position.x
    def armPose_callback(self, msg):
        self.armPose = msg.data 
    def points_callback(self, msg):
        self.elements.append([msg.x, msg.y])
        rospy.loginfo(self.elements)

if __name__ == '__main__':
    try:
        rospy.init_node('patin_control', anonymous=True)
        patineta = patineta()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
