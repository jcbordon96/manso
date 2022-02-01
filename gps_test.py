#!/usr/bin/env python3
from datetime import datetime, timedelta
from ublox_gps import UbloxGps
import serial
import rospy
from sensor_msgs.msg  import NavSatFix, NavSatStatus
from std_msgs.msg import Time, String
from datetime import datetime, timedelta
# Can also use SPI here - import spidev
# I2C is not supported

port = serial.Serial('/dev/GPS', baudrate=9600, timeout=1)
gps = UbloxGps(port)
gnss = NavSatFix()
gnss.header.frame_id = "base_link"
time = String()

class Run():
    def __init__(self):
        self.gnss_pub = rospy.Publisher("gnss", NavSatFix, queue_size=50)
        self.time_pub = rospy.Publisher("gnss_time", String, queue_size=50)
        try:
            print("Listenting for UBX Messages.")
            while not rospy.is_shutdown():
                try:
                    coords = gps.geo_coords()
                    
                    gnss.header.stamp = rospy.Time.now()
                    gnss.latitude = round(coords.lat,7)
                    gnss.longitude = round(coords.lon,7)
                    gnss.altitude = coords.height
                    utc = datetime.strptime(str(coords.year).zfill(2)+"-"+str(coords.month).zfill(2)+"-"+str(coords.day).zfill(2)+" "+str(coords.hour).zfill(2)+":"+str(coords.min).zfill(2)+":"+str(coords.sec).zfill(2), '%Y-%m-%d %H:%M:%S')
                    gmt = utc - timedelta(hours=3)
                    gmt_str = gmt.strftime('%Y%m%d_%H%M%S')
                    print(gmt_str)
                    time.data = gmt_str
                    self.time_pub.publish(time)
                    self.gnss_pub.publish(gnss)

                    print(gmt_str+"_"+str(round(coords.lat,7))+"_"+str(round(coords.lon,7))+"_"+"x")
                    # print(coords.lon, coords.lat)
                except (ValueError, IOError) as err:
                    print(err)

        finally:
            port.close()


if __name__ == '__main__':
    try:
        rospy.init_node('GNSS')
        Run()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Chau")
