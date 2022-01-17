#!/usr/bin/env python3
# Python code for Multiple Color Detection
from fileinput import close
import rospy
from std_msgs.msg import Bool
import numpy as np
import cv2
from geometry_msgs.msg import Twist


class Follower:
    msg = Twist()
    angular = 0.1
    linear = 0.40
    request = True
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('follower_request', Bool, self.follower_request_callback)
        flagStop = False
        # Capturing video through webcam
        webcam = cv2.VideoCapture(0)
        webcam.set(3, 640)  # width=1920
        webcam.set(4, 480) 
            # Start a while loop
        self.status = 0
        framesEmpty = 0
        closent = True
        while closent :
            try:
                framesEmpty = framesEmpty + 1
                # Reading the video from the
                # webcam in image frames
                _, imageFrame = webcam.read()
            
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
                green_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
            
                
                # Morphological Transform, Dilation
                # for each color and bitwise_and operator
                # between imageFrame and mask determines
                # to detect only that particular color
                kernal = np.ones((5, 5), "uint8")
                
                # For blue color
                green_mask = cv2.dilate(green_mask, kernal)
                res_green = cv2.bitwise_and(imageFrame, imageFrame, 
                                        mask = green_mask)
                
                # Creating contour to track blue color
                contours, hierarchy = cv2.findContours(green_mask,
                                                    cv2.RETR_TREE,
                                                    cv2.CHAIN_APPROX_SIMPLE)
                
                for pic, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if(area > 2000):
                        flagStop = False
                        framesEmpty = 0
                        peri = cv2.arcLength(contour, True)
                        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
                        cv2.drawContours(imageFrame, contour, -1, (255,0,255), 1)
                        x, y, w, h = cv2.boundingRect(contour)
                        cv2.rectangle(imageFrame, (x,y), (x+w, y+h), (0,0,255), 3)
                        self.pubCorrection(x)

                        #print(area)
                        #print(len(approx))
                        print(x)
                if framesEmpty >= 40:
                    self.pubCorrection(-1)
                    flagStop = True
                if(flagStop):
                    self.pubCorrection(-1)
                # Program Termination
                cv2.rectangle(imageFrame, (225,0), (225, 480), (255,0,255), 3)
                cv2.rectangle(imageFrame, (415,0), (415, 480), (255,0,255), 3)
                cv2.rectangle(imageFrame, (0,300), (640, 300), (255,0,255), 3)
                cv2.imshow("Follower", imageFrame)
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    closent = False
                    cap.release()
                    cv2.destroyAllWindows()
                    break
            except:
                print("except")
                pass
    def follower_request_callback(self, msg):
        self.request = msg.data
        if self.request == False:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.pub.publish(self.msg)
    def pubCorrection(self, value):
            if self.request == True:
                if(self.status != value):
                    print(self.status)
                    if value == -1:
                        self.msg.linear.x = 0.0
                        self.msg.angular.z = 0.0
                    elif value < 220:
                        self.msg.linear.x = self.linear
                        self.msg.angular.z = self.angular
                    elif value > 420:
                        self.msg.linear.x = self.linear
                        self.msg.angular.z = - self.angular
                    else: 
                        self.msg.linear.x = self.linear
                        self.msg.angular.z = 0
                    self.pub.publish(self.msg)
                    self.status = value
def main():
    rospy.init_node('Follower', anonymous=True)
    ic = Follower()

if __name__ == '__main__':
    main()
