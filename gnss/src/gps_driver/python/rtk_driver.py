#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from gps_driver.msg import Customrtk  # Updated message type
import utm
import pynmea2

class RTKDriver:
    def __init__(self, file_path):
        rospy.init_node('rtk_driver', anonymous=True)
        self.rtk_pub = rospy.Publisher('/rtk', Customrtk, queue_size=10)
        self.file_path = file_path
        self.read_and_publish_data()

    def read_and_publish_data(self):
        with open(self.file_path, 'r') as file:
            for line in file:
                if line.startswith('$GNGGA'):  # Filter out lines that do not start with "$GNGGA"
                    try:
                        sentence = pynmea2.parse(line)
                        if isinstance(sentence, pynmea2.types.talker.GGA):
                            msg = self.create_custom_rtk_message(sentence)
                            self.rtk_pub.publish(msg)
                            rospy.loginfo("Published data: %s", msg)
                    except pynmea2.ParseError:
                        continue

    def create_custom_rtk_message(self, sentence):
        msg = Customrtk()
        msg.header = Header()
        msg.header.frame_id = 'RTK_Frame'
        msg.header.stamp = rospy.Time.now()

        msg.latitude = sentence.latitude
        msg.longitude = sentence.longitude

        altitude_str = sentence.altitude
        msg.altitude = float(altitude_str) if altitude_str else 0.0

        utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
        msg.utm_easting = utm_coords[0]
        msg.utm_northing = utm_coords[1]
        msg.zone = utm_coords[2]
        msg.letter = utm_coords[3]

        msg.fix_quality = sentence.gps_qual  # Add fix quality

        # Convert hdop to float
        hdop_str = sentence.horizontal_dil
        msg.hdop = float(hdop_str) if hdop_str else 0.0

        msg.gpgga_read = line.strip()  # Store GNGGA string

        return msg

if __name__ == '__main__':
    try:
        file_path = rospy.get_param('~port', '/dev/pts/4')  # Specify file path parameter
        rospy.loginfo("Selected file path: %s", file_path)
        RTKDriver(file_path)
    except rospy.ROSInterruptException:
        pass

