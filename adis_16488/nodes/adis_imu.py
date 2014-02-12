#!/usr/bin/env python

import serial
import string
import math

import roslib; roslib.load_manifest('adis_16488')
import rospy
import tf
from sensor_msgs.msg import Imu
from adis_16488.msg import ADIS16488

IMU_FRAME = 'imu_link'
grad2rad = 3.141592654/180.0

if __name__ == '__main__': 
    rospy.init_node('adis_16488') 
    imu_pub = rospy.Publisher(rospy.get_name()+'/imu_data_raw', Imu)
    adis_info_pub = rospy.Publisher(rospy.get_name()+'/adis_16488_info', ADIS16488)        
    port = rospy.get_param("~port", "/dev/ttyACM0")
    baud = int(rospy.get_param("~baud", "115200"))
    ser = serial.Serial(port, baud, timeout=1)
    rospy.loginfo('Publishing imu data on topics:\n'+rospy.get_name()+'/imu_data_raw\n'+'/adis_16488_info')

    imuMsg = Imu()
    ADISInfo = ADIS16488()
    imuMsg.orientation_covariance = [999999 , 0 , 0, 0, 9999999, 0, 0, 0, 999999]
    imuMsg.angular_velocity_covariance = [9999, 0 , 0, 0 , 99999, 0, 0 , 0 , 0.02]
    imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0, 0 , 0.2, 0, 0 , 0 , 0.2]

    yaw = 0.0
    pitch = 0.0
    roll = 0.0
    prod_id = 0.0
    rospy.sleep(3)

    while not rospy.is_shutdown():
        line = ser.readline()
        words = string.split(line,",")
        #print words
        if len(words) == 15:
            try:
                prod_id = int(words[0])
                yaw = float(words[1])*grad2rad
                pitch = -float(words[2])*grad2rad
                roll = -float(words[3])*grad2rad               

                imuMsg.linear_acceleration.x = float(words[4])*0.8*0.001*9.8
                imuMsg.linear_acceleration.y = float(words[5])*0.8*0.001*9.8
                imuMsg.linear_acceleration.z = float(words[6])*0.8*0.001*9.8

                ADISInfo.X_MAGN_OUT = float(words[7])
                ADISInfo.Y_MAGN_OUT = float(words[8])
                ADISInfo.Z_MAGN_OUT = float(words[9])

                imuMsg.angular_velocity.x = float(words[10])*0.02*grad2rad
                imuMsg.angular_velocity.y = float(words[11])*0.02*grad2rad
                imuMsg.angular_velocity.z = float(words[12])*0.02*grad2rad
            
                ADISInfo.X_ACCL_OUT = float(words[4])
                ADISInfo.Y_ACCL_OUT = float(words[5])
                ADISInfo.Z_ACCL_OUT = float(words[6])
                ADISInfo.X_GYRO_OUT = float(words[10]) 
                ADISInfo.Y_GYRO_OUT = float(words[11])
                ADISInfo.Z_GYRO_OUT = float(words[12])
            
                ADISInfo.TEMP_OUT = float(words[13])
                ADISInfo.BAROM_OUT = float(words[14])
            except Exception as e:
                print 'value error', e
        
        q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        imuMsg.orientation.x = q[0]
        imuMsg.orientation.y = q[1]
        imuMsg.orientation.z = q[2]
        imuMsg.orientation.w = q[3]
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = IMU_FRAME
        imu_pub.publish(imuMsg)

        ADISInfo.prod_id = prod_id
        ADISInfo.yaw = yaw
        ADISInfo.pitch = pitch
        ADISInfo.roll = roll
        adis_info_pub.publish(ADISInfo)
        #rospy.sleep(0.05)
    ser.close()        
    
      
