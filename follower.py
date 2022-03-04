#!/usr/bin/env python3
# Python code for Multiple Color Detection
from fileinput import close

import rospy
from std_msgs.msg import Bool
import numpy as np
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import os
import math
import time as ptime

class Follower:
    msg = Twist()
    #max_angular = 0.08
    #max_linear = 0.25
    #min_angular = 0.05
    #min_linear = 0.12
    max_angular = 0.15
    max_linear = 0.45
    min_angular = 0.15
    min_linear = 0.2
    request = True
    vois = Int16()
    fileTime=int(ptime.time())
    os.mkdir('/home/appelie/manso_ws/src/v1/scripts/manso/resources/videos/follower/{}'.format(fileTime))
    out_debug = cv2.VideoWriter('/home/appelie/manso_ws/src/v1/scripts/manso/resources/videos/follower/{}/debug.avi'.format(fileTime), cv2.VideoWriter_fourcc(*'MJPG'),10, (640,480))
    out_raw = cv2.VideoWriter('/home/appelie/manso_ws/src/v1/scripts/manso/resources/videos/follower/{}/raw.avi'.format(fileTime), cv2.VideoWriter_fourcc(*'MJPG'),10, (640,480))

    def __init__(self):
        os.system('v4l2-ctl -d /dev/FOLLOWER -c exposure_auto=1')
        os.system('v4l2-ctl -d /dev/FOLLOWER -c exposure_absolute=1')
        os.system('v4l2-ctl -d /dev/FOLLOWER -c white_balance_temperature_auto=0')
        os.system('v4l2-ctl -d /dev/FOLLOWER -c white_balance_temperature=5000')
        os.system('v4l2-ctl -d /dev/FOLLOWER -c focus_auto=0')
        os.system('v4l2-ctl -d /dev/FOLLOWER -c focus_absolute=12')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('follower_request', Bool, self.follower_request_callback)
        

        self.voice_pub = rospy.Publisher('voice_cmd', Int16, queue_size=10)
        flagStop = False
        # Capturing video through webcam
        webcam = cv2.VideoCapture("/dev/FOLLOWER")
        webcam.set(3, 640)  # width=1920
        webcam.set(4, 480) 
            # Start a while loop
        self.status = 0
        framesEmpty = 0
        closent = True
        exposureValue = 1
        flagChangeExposure = False
        flagChangeExposureDecrease = False
        angle = 0
        
        while closent :
            try:
                framesEmpty = framesEmpty + 1
                # Reading the video from the
                # webcam in image frames
                _, imageFrame = webcam.read()
                self.out_raw.write(imageFrame)
            
                # Convert the imageFrame in 
                # BGR(RGB color space) to 
                # HSV(hue-saturation-value)
                # color space
                hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
            
                # Set range for red color and 
                # define mask
                red_lower = np.array([136, 87, 111], np.uint8)
                red_upper = np.array([180, 255, 255], np.uint8)
                #blue_lower = np.array([110,50,50])
                #blue_upper = np.array([130,255,255])
                #green_lower = np.array([25, 52, 72], np.uint8
                #)
                #green_upper = np.array([102, 255, 255], np.uint8)
                mask = cv2.inRange(hsvFrame, red_lower, red_upper)
            
                
                # Morphological Transform, Dilation
                # for each color and bitwise_and operator
                # between imageFrame and mask determines
                # to detect only that particular color
                kernal = np.ones((5, 5), "uint8")
                
                # For blue color
                mask = cv2.dilate(mask, kernal)
                
                # Creating contour to track blue color
                contours, hierarchy = cv2.findContours(mask,
                                                    cv2.RETR_TREE,
                                                    cv2.CHAIN_APPROX_SIMPLE)
                
                for pic, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    x, y, w, h = cv2.boundingRect(contour)
                    #print(area)
                    if(area > 10000):
                        
                        flagStop = False
                        if(flagChangeExposure):
                            if (flagChangeExposureDecrease):
                                os.system('v4l2-ctl -d /dev/FOLLOWER -c exposure_absolute={}'.format(exposureValue))
                            flagChangeExposure = False
                        framesEmpty = 0
                        cv2.drawContours(imageFrame, contour, -1, (255,0,255), 1)
                        M = cv2.moments(contour)
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.circle(imageFrame, (cx,cy), 10, (255,255,255), cv2.FILLED)
            

                        if(x > 270 and x+w < 370):
                            cv2.rectangle(imageFrame, (x,y), (x+w, y+h), (0,255,0), 3)
                        else:
                            cv2.rectangle(imageFrame, (x,y), (x+w, y+h), (0,0,255), 3)
                        height,width = mask.shape
                        skel = np.zeros([height,width],dtype=np.uint8)      #[height,width,3]
                        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
                        while(np.count_nonzero(mask) != 0 ):
                            eroded = cv2.erode(mask,kernel)
                            # cv2.imshow("eroded",eroded)   
                            temp = cv2.dilate(eroded,kernel)
                            # cv2.imshow("dilate",temp)
                            temp = cv2.subtract(mask,temp)
                            skel = cv2.bitwise_or(skel,temp)
                            mask = eroded.copy()
                        
                        # cv2.imshow("skel",skel)
                        edges = cv2.Canny(skel, 50, 150)
                        # cv2.imshow("edges",edges)
                        lines = cv2.HoughLinesP(edges,1,np.pi/180,40,minLineLength=30,maxLineGap=30)
                        i = 0

                        if( type(lines) == np.ndarray):
                            for x1,y1,x2,y2 in lines[0]:
                                i+=1
                                dx = x1 - x2
                                dy = y1 - y2 
                                angle = math.atan((dx/dy))
                                angle = round((angle/(2*math.pi))*360,2)
                                cv2.putText(imageFrame, "{}".format(angle), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA, False)
                    
                        self.pubCorrection(cx,angle)

                        #print(area)
                        #print(len(approx))
                        # print(x)
                if framesEmpty % 5 == 0 and flagChangeExposure:
                    if(flagChangeExposure):
                        if(exposureValue < 10 and not flagChangeExposureDecrease):
                            exposureValue += 1
                        else:
                            exposureValue -= 1
                        if(exposureValue == 10):
                            flagChangeExposureDecrease = True
                        elif (exposureValue == 1):
                            flagChangeExposureDecrease = False
                        os.system('v4l2-ctl -d /dev/FOLLOWER -c exposure_absolute={}'.format(exposureValue))
                if framesEmpty >= 40:
                    self.pubCorrection(-1,angle)
                    flagStop = True
                    flagChangeExposure = True
                    framesEmpty = 1
                if(flagStop):
                    self.pubCorrection(-1, angle)
                
                # Program Termination
                cv2.rectangle(imageFrame, (270,0), (270, 480), (255,0,255), 3)
                cv2.rectangle(imageFrame, (370,0), (370, 480), (255,0,255), 3)
                cv2.rectangle(imageFrame, (10,10), (200, 50), (0,0,0), -1)
                cv2.putText(imageFrame, "Linear: {}".format(self.msg.linear.x), (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255), 3,3)
                cv2.rectangle(imageFrame, (350,10), (630, 50), (0,0,0), -1)
                cv2.putText(imageFrame, "Angular: {}".format(self.msg.angular.z), (350,40), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255), 3,3)
                self.out_debug.write(imageFrame)
                cv2.imshow("Follower", imageFrame)
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    closent = False
                    cap.release()
                    cv2.destroyAllWindows()
                    break
            except Exception as e:
                print(e)
                pass
    def follower_request_callback(self, msg):
        self.request = msg.data
        if self.request == False:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.pub.publish(self.msg)
    def pubCorrection(self, value, angle):
            if self.request == True:
                # if(self.status != value):
                #     print(value)
                # # print(self.status)
                if value == -1:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.vois.data = -1
                    self.voice_pub.publish(self.vois)
                elif value < 270 and angle >= 0 :
                    self.msg.linear.x = self.min_linear + (self.max_linear - self.min_linear) * (value/270)
                    self.msg.angular.z = max(self.min_angular,(self.max_angular - self.max_angular * (value/270)))
                    self.vois.data = 1
                    self.voice_pub.publish(self.vois)
                elif value < 270 and angle < 0 :
                    self.msg.linear.x = self.min_linear 
                    self.msg.angular.z = 0
                    self.vois.data = 1
                    self.voice_pub.publish(self.vois)
                elif value > 370 and angle <= 0:
                    self.msg.linear.x = self.max_linear - (self.max_linear - self.min_linear) * ((value - 370)/270)
                    self.msg.angular.z = min(-self.min_angular, (-self.max_angular * ((value - 370)/270)))
                    self.vois.data = 2
                    self.voice_pub.publish(self.vois)
                elif value > 370 and angle > 0:
                    self.msg.linear.x = self.min_linear
                    self.msg.angular.z = 0
                    self.vois.data = 2
                    self.voice_pub.publish(self.vois)
                else: 
                    self.msg.linear.x = self.max_linear - (self.max_linear - self.min_linear) * (min(abs(angle),7.0)/7.0)
                    self.msg.angular.z = (((angle/abs(angle))*(min(3.0,abs(angle)))) / 3.0) * self.max_angular
                    self.vois.data = 0
                    self.voice_pub.publish(self.vois)
                        
                self.pub.publish(self.msg)
                if(self.status != self.vois.data):
                    print(self.vois.data)
                self.status = self.vois.data
def main():
    rospy.init_node('Follower', anonymous=True)
    ic = Follower()

if __name__ == '__main__':
    main()
