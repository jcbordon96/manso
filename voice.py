#!/usr/bin/env python3
import rospy
import std_msgs
from std_msgs.msg import Int16
import os


class Voice:
    def __init__(self):
        self.cmd = 0
        self.compensationValue = 0

        def callback(msg):
            self.cmd = msg.data

        rospy.Subscriber('voice_cmd', Int16, callback)
        while not rospy.is_shutdown():
            if(self.cmd == -1 and self.compensationValue != -1):
                os.system("mpg123 /home/appelie/manso_ws/src/v1/scripts/manso/resources/voice/detenido.mp3")
                self.compensationValue = -1
            elif(self.cmd == 1  and self.compensationValue != 1):
                os.system("mpg123 /home/appelie/manso_ws/src/v1/scripts/manso/resources/voice/com_izq.mp3")
                self.compensationValue = 1
            elif(self.cmd == 0 and self.compensationValue != 0):
                os.system("mpg123 /home/appelie/manso_ws/src/v1/scripts/manso/resources/voice/alineado.mp3")
                self.compensationValue = 0
            elif(self.cmd == 2 and self.compensationValue != 2):
                os.system("mpg123 /home/appelie/manso_ws/src/v1/scripts/manso/resources/voice/com_der.mp3")
                self.compensationValue = 2
            elif(self.cmd == 3 and self.compensationValue != 3):
                os.system("mpg123 /home/appelie/manso_ws/src/v1/scripts/manso/resources/voice/easter0.mp3")
                self.compensationValue = 3
            elif(self.cmd == 4 and self.compensationValue != 4):
                os.system("mpg123 /home/appelie/manso_ws/src/v1/scripts/manso/resources/voice/easter1.mp3")
                self.compensationValue = 4
    
def main():
    rospy.init_node('Voice', anonymous=True)
    ic = Voice()

if __name__ == '__main__':
    main()
