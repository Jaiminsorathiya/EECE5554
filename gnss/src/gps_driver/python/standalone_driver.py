#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from gps_driver.msg import Customgps
import utm
import serial
import pynmea2
from datetime import datetime

class GNSSDriver:
    def __init__(self, port):
        rospy.init_node('gnss_driver', anonymous=True)
        self.gps_pub = rospy.Publisher('/gps', Customgps, queue_size=10)
        self.ser = serial.Serial(port, baudrate=4800, timeout=1)

    def read_data(self):
        while not rospy.is_shutdown():
            data = self.ser.readline().decode('utf-8').strip()
            print("Received data:", data)  # Add this line for debugging

            if not data:
                continue

            try:
                msg = self.parse_nmea_sentence(data)
                self.gps_pub.publish(msg)
                print("Published data:", msg)  # Add this line for debugging
            except pynmea2.ParseError as e:
                print(f"Parse error: {e}")
                continue

    def parse_nmea_sentence(self, data):
        msg = Customgps()
        msg.header = Header()
        msg.header.frame_id = 'GPS1_Frame'
        msg.header.stamp = rospy.Time.now()

        sentence = pynmea2.parse(data)

        if isinstance(sentence, pynmea2.types.talker.GGA):
            msg.latitude = sentence.latitude
            msg.longitude = sentence.longitude

            # Handle altitude parsing
            altitude_str = sentence.altitude
            msg.altitude = float(altitude_str) if altitude_str else 0.0

            utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
            msg.utm_easting = utm_coords[0]
            msg.utm_northing = utm_coords[1]
            msg.zone = utm_coords[2]
            msg.letter = utm_coords[3]

            # Handle HDOP parsing
            hdop_str = sentence.horizontal_dil
            msg.hdop = float(hdop_str) if hdop_str else 0.0

            msg.gpgga_read = data
        elif isinstance(sentence, pynmea2.types.talker.RMC):
            msg.gprmc_read = data

        return msg

def main():
    try:
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        print("Selected port:", port)
        gnss_driver = GNSSDriver(port)
        gnss_driver.read_data()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

