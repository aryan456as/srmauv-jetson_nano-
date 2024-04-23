#!/usr/bin/env python
# Software License Agreement (BSD License)

# Sparton Digital Compass ROS Driver for AHRS-8/GEDC-6
# Copyright (c) 2013, Cheng-Lung Lee, University of Detroit Mercy.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Sparton Digital Compass ROS Driver for AHRS-8/GEDC-6
# Copyright (c) 2013, Cheng-Lung Lee, University of Detroit Mercy.

# Changelog

# 2013.01.06 Add IMU message
# 2012.12.13 Use Pos2D message, normalized to 0 ~ 2*PI
#

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import serial
import math
import time

def wrapTo2PI(theta):
    '''Normalize an angle in radians to [0, 2*pi]'''
    return theta % (2.*math.pi)

def wrapToPI(theta):
    '''Normalize an angle in radians to [-pi, pi]'''
    return (wrapTo2PI(theta+math.pi) - math.pi)

def Spartonshutdownhook():
    global D_Compass
    global myStr1
    print "Sparton shutdown time!"
    D_Compass.write(myStr1)  # stop data stream before close port
    D_Compass.flush()  # flush data out
    rospy.loginfo('Closing Digital Compass Serial port')
    D_Compass.close()  # Close D_Compass serial port

if __name__ == '__main__':
    global D_Compass
    global myStr1
    rospy.init_node('SpartonDigitalCompassIMU')
    Pos_pub = rospy.Publisher('imu/HeadingTrue', Pose2D, queue_size=10)
    PosD_pub = rospy.Publisher('imu/HeadingTrue_degree', Pose2D, queue_size=10)
    PosD_pubD = rospy.Publisher('imu/Heading_degree/theta', Float64, queue_size=10)
    Roll_pub = rospy.Publisher('imu/roll', Float64, queue_size=10)
    Pitch_pub = rospy.Publisher('imu/pitch', Float64, queue_size=10)
    Imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
    SpartonPose2D = Pose2D()
    SpartonPose2D.x = float(0.0)
    SpartonPose2D.y = float(0.0)
    SpartonPose2D_D = SpartonPose2D

    # Init D_Compass port
    D_Compassport = rospy.get_param('~port', '/dev/ttyUSB0')
    D_Compassrate = rospy.get_param('~baud', 115200)
    D_Compassprintmodulus = rospy.get_param('~printmodulus', 1)
    D_Compass_declination = rospy.get_param('~declination', -7.462777777777778) * (math.pi/180.0)
    D_Compass_UseEastAsZero = rospy.get_param('~UseEastAsZero', True)
    Checksum_error_limits = rospy.get_param('~Checksum_error_limits', 10.)
    checksum_error_counter = 0
    myStr1 = '\r\n\r\nprinttrigger 0 set drop\r\n'
    myStr2 = 'printmask gyrop_trigger accelp_trigger or quat_trigger or time_trigger or set drop\r\n'
    myStr_printmodulus = ('printmodulus %i set drop\r\n' % D_Compassprintmodulus)
    myStr3 = 'printtrigger printmask set drop\r\n'

    rospy.on_shutdown(Spartonshutdownhook)

    try:
        D_Compass = serial.Serial(port=D_Compassport, baudrate=D_Compassrate, timeout=.5)
        D_Compass.write(myStr1)
        D_Compass.flush()
        time.sleep(0.5)
        if (D_Compass.inWaiting() > 0):
            data = D_Compass.read(D_Compass.inWaiting())
            print("Send to Digital Compass: %s Got: %s" % (myStr1, data))
        else:
            rospy.logerr('[Sparton][1]Received No data from DigitalCompass. Shutdown!')
            rospy.signal_shutdown('Received No data from DigitalCompass')

        D_Compass.write(myStr2)
        data = D_Compass.readline()
        if (len(data) > 0):
            rospy.loginfo("Send to Digital Compass: %s" % myStr2)
            rospy.loginfo("Send to Digital Compass. Got: %s" % data)
            D_Compass.write(myStr_printmodulus)
            data = D_Compass.readline()
            rospy.loginfo("Send to Digital Compass: %s " % myStr_printmodulus)
            rospy.loginfo("Send to Digital Compass. Got: %s" % data)
            D_Compass.write(myStr3)
            data = D_Compass.readline()
            rospy.loginfo("Send to Digital Compass: %s " % myStr3)
            rospy.loginfo("Send to Digital Compass. Got: %s" % data)
            rospy.loginfo('Digital Compass Setup Complete!')
        else:
            rospy.logerr('[Sparton][2]Received No data from DigitalCompass. Shutdown!')
            rospy.signal_shutdown('Received No data from DigitalCompass')

        while not rospy.is_shutdown():
            data = D_Compass.readline()
            DataTimeSec = rospy.get_time()
            fields = data.split(',')
            try:
                if len(fields) > 14:
                    if 'P:apgpq' == (fields[0] + fields[2] + fields[6] + fields[10]):
                        Ax = float(fields[3]) / 1000. * 9.81
                        Ay = float(fields[4]) / 1000. * 9.81
                        Az = float(fields[5]) / 1000. * 9.81
                        Gx = float(fields[7]) * (math.pi/180.0)
                        Gy = float(fields[8]) * (math.pi/180.0)
                        Gz = float(fields[9]) * (math.pi/180.0)
                        w = float(fields[11])
                        x = float(fields[12])
                        y = float(fields[13])
                        z = float(fields[14])

                        imu_data = Imu()
                        imu_data.header.stamp = rospy.Time.from_sec(DataTimeSec - len(data) / 11520.)
                        imu_data.orientation = Quaternion()
                        imu_data.orientation.x = y
                        imu_data.orientation.y = x
                        imu_data.orientation.z = -z
                        imu_data.orientation.w = w

                        ENU = [y, x, -z, w]
                        angle_ENU = euler_from_quaternion(ENU, axes='sxyz')
                        angle_ROS = (angle_ENU[0], angle_ENU[1], angle_ENU[2] + math.pi/2. - D_Compass_declination)
                        q_ROS = quaternion_from_euler(angle_ROS[0], angle_ROS[1], angle_ROS[2], axes='sxyz')

                        (r, p, y) = euler_from_quaternion([imu_data.orientation.x, imu_data.orientation.y,
                                                            imu_data.orientation.z, imu_data.orientation.w])
                        Roll_pub.publish(r)
                        Pitch_pub.publish(p)

                        if D_Compass_UseEastAsZero:
                            (imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z,
                             imu_data.orientation.w) = q_ROS
                        else:
                            pass

                        imu_data.angular_velocity.x = Gy
                        imu_data.angular_velocity.y = Gx
                        imu_data.angular_velocity.z = -Gz
                        imu_data.linear_acceleration.x = Ay
                        imu_data.linear_acceleration.y = Ax
                        imu_data.linear_acceleration.z = -Az

                        Imu_pub.publish(imu_data)

                        SpartonPose2D.theta = wrapToPI(angle_ROS[2])
                        Pos_pub.publish(SpartonPose2D)
                        SpartonPose2D_D.theta = SpartonPose2D.theta / math.pi * 180.
                        PosD_pub.publish(SpartonPose2D_D)
                        PosD_pubD.publish(SpartonPose2D_D.theta)

                        checksum_error_counter = 0

                    else:
                        rospy.logerr("[Sparton][3]Received a sentence but not correct. Sentence was: %s" % data)
                        checksum_error_counter += 1
                        if (checksum_error_counter > Checksum_error_limits):
                            rospy.logfatal('[Sparton Compass] Too much back to back checksumn error in Sparton Compass data. Shutdown!')
                            rospy.signal_shutdown('Too much back to back checksum error in Sparton Compass data')

                else:
                    rospy.logerr("[Sparton][4]Received a sentence but not correct. Sentence was: %s" % data)
                    checksum_error_counter += 1
                    if (checksum_error_counter > Checksum_error_limits):
                        rospy.logfatal('[Sparton Compass] Too much back to back checksumn error in Sparton Compass data. Shutdown!')
                        rospy.signal_shutdown('Too much back to back checksum error in Sparton Compass data')

            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the data messages.Sentence was: %s" % data)

        D_Compass.write(myStr1)
        D_Compass.flush()
        rospy.loginfo('Closing Digital Compass Serial port')
        D_Compass.close()

    except rospy.ROSInterruptException:
        pass
