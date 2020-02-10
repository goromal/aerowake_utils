#!/usr/bin/python

import rospy, subprocess
import time

if __name__ == '__main__':

    # initialize
    rospy.init_node('imu_calibrator')
    rospy.loginfo("USING AUTOMATIC IMU CALIBRATOR FOR SIM TESTING")

    wait_secs = int(rospy.get_param('~wait_secs', 1))
    rate = rospy.Rate(1)
    for i in range(0, wait_secs):
        rate.sleep()

    rospy.loginfo('ATTEMPTING TO CALIBRATE IMU AFTER %d SECONDS' % wait_secs)
    subprocess.call(['rosservice','call','/calibrate_imu'])

    for i in range(0, 4):
        rate.sleep()
