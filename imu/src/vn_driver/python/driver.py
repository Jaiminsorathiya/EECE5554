#!/usr/bin/env python3
import rospy
import serial
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
from vn_driver.msg import Vectornav as VNMessage

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return qx, qy, qz, qw

seq_count = 0  # Initialize a sequence counter at the beginning of your script

def imu_driver():
    global seq_count
    rospy.init_node('imu_driver_node', anonymous=True)
    
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baud_rate = rospy.get_param('~baudrate', 115200)
    publisher = rospy.Publisher('imu_data', VNMessage, queue_size=10)

    ser = serial.Serial(port, baud_rate, timeout=3)
    ser.write(b"$VNWRG,07,40*xx")
    
    while not rospy.is_shutdown():
        received_data = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if "$VNYMR" in received_data:
            data = received_data.split(",")
            now_time = rospy.get_rostime()

            if len(data) >= 13:
                yaw, pitch, roll = [np.radians(float(data[i])) for i in range(1, 4)]
                magX, magY, magZ = [float(data[i]) for i in range(4, 7)]
                accX, accY, accZ = [float(data[i]) for i in range(7, 10)]
                gyroX, gyroY, gyroZ = [float(data[i].split('')[0]) if '' in data[i] else float(data[i]) for i in range(10, 13)]

                qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

                vn_msg = VNMessage()
                vn_msg.header = Header(seq=seq_count, stamp=now_time, frame_id="IMU_Frame")

                imu_msg = Imu()
                imu_msg.header = Header(seq=seq_count, stamp=now_time, frame_id="IMU_Frame")
                imu_msg.orientation.x = qx
                imu_msg.orientation.y = qy
                imu_msg.orientation.z = qz
                imu_msg.orientation.w = qw
                imu_msg.linear_acceleration.x = accX
                imu_msg.linear_acceleration.y = accY
                imu_msg.linear_acceleration.z = accZ
                imu_msg.angular_velocity.x = gyroX
                imu_msg.angular_velocity.y = gyroY
                imu_msg.angular_velocity.z = gyroZ

                mag_msg = MagneticField()
                mag_msg.header = Header(seq=seq_count, stamp=now_time, frame_id="IMU_Frame")
                mag_msg.magnetic_field.x = magX
                mag_msg.magnetic_field.y = magY
                mag_msg.magnetic_field.z = magZ

                vn_msg.imu = imu_msg
                vn_msg.mag_field = mag_msg
                vn_msg.raw_data = received_data

                publisher.publish(vn_msg)
                seq_count += 1

if __name__ == "__main__":
    try:
        imu_driver()
    except rospy.ROSInterruptException:
        pass

