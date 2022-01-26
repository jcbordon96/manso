#!/usr/bin/env python
import rospy
import std_msgs
from std_msgs.msg import Int16
import os


class Voice:
    def __init__(self):
        self.pixel = 0
        self.compensationValue = 0

        def callback(msg):
            self.pixel = msg.data

        rospy.Subscriber('manguera_cmd', Int16, callback)
        while(1):
            if(self.pixel == -1 and self.compensationValue != -1):
                os.system("mpg123 /home/appelie/follow_ws/src/voice/scripts/detenido.mp3")
                self.compensationValue = -1
            elif(self.pixel >= 0 and self.pixel < 225 and self.compensationValue != 1):
                os.system("mpg123 /home/appelie/follow_ws/src/voice/scripts/com_izq.mp3")
                self.compensationValue = 1
            elif(self.pixel > 225 and self.pixel < 415 and self.compensationValue != 0):
                os.system("mpg123 /home/appelie/follow_ws/src/voice/scripts/alineado.mp3")
                self.compensationValue = 0
            elif(self.pixel > 415 and self.compensationValue != 2):
                os.system("mpg123 /home/appelie/follow_ws/src/voice/scripts/com_der.mp3")
                self.compensationValue = 2
    
def main():
    rospy.init_node('Voice', anonymous=True)
    ic = Voice()

if __name__ == '__main__':
    main()
