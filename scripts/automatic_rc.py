#!/usr/bin/python

import rospy, subprocess, yaml, time
from rosflight_msgs.msg import RCRaw
import time

if __name__ == '__main__':

    # ENUMS
    HIGH = 2000
    MED  = 1500
    LOW  = 1000

    # initialize
    rospy.init_node('rc_joy')
    rc_pub = rospy.Publisher('RC', RCRaw, queue_size=10)
    rospy.loginfo("USING AUTOMATIC RC ARM FUNCTIONALITY")

    # GET ROSFLIGHT SWITCH CHANNELS
    RC_THR_CHN = None #2-
    RC_OVD_CHN = None #4-
    RC_ARM_CHN = None #5-
    rfpfile = rospy.get_param('~rosflight_paramfile')
    with open(rfpfile) as f:
        data = yaml.load(f, Loader=yaml.Loader)
        for datum in data:
            if datum['name'] == 'RC_F_CHN':
                RC_THR_CHN = datum['value']
            if datum['name'] == 'RC_THR_OVRD_CHN':
                RC_OVD_CHN = datum['value']
            if datum['name'] == 'ARM_CHANNEL':
                RC_ARM_CHN = datum['value']

    sleep_time = 0.05
    sleep_rate = 1.0 / sleep_time
    rate = rospy.Rate(sleep_rate)
    rateQsec = rospy.Rate(4)
    rate2sec = rospy.Rate(0.5)
    wait_secs = int(rospy.get_param('~wait_secs', 1))

    msg = RCRaw()
    # Values should range from 1000 - 2000
    msg.values[0] = MED
    msg.values[1] = MED
    msg.values[RC_THR_CHN] = LOW
    msg.values[3] = MED
    msg.values[RC_OVD_CHN] = HIGH
    msg.values[RC_ARM_CHN] = LOW
    msg.values[6] = MED
    msg.values[7] = MED

    idx = 0
    uninitiated = True
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        time_i = idx * sleep_time

        if time_i >= wait_secs and uninitiated:
            rospy.loginfo('SETTING ARM AND OVERRIDE CHANNELS TO HIGH')
            msg.values[RC_ARM_CHN] = HIGH
            msg.values[RC_OVD_CHN] = LOW
            rc_pub.publish(msg)
            # rateQsec.sleep()
            # rateQsec.sleep()

            # rate2sec.sleep()
            time.sleep(2.0)

            rospy.loginfo('SETTING THROTTLE HIGH')
            msg.values[RC_THR_CHN] = HIGH
            uninitiated = False

        rc_pub.publish(msg)
        rate.sleep()
        idx += 1




    #
    #
    # # calibrate IMU
    #
    # # main loop
    # update_freq = 10 # Hz
    # BASE_SECS = 5
    # barocal_count = (BASE_SECS + 5) * update_freq
    # calibrate_count = BASE_SECS * update_freq
    # switch_count = (BASE_SECS + 10) * update_freq
    # throttle_count = (BASE_SECS + 11) * update_freq
    #
    # rate = rospy.Rate(update_freq)
    #
    # msg = RCRaw()
    # # Values should range from 1000 - 2000
    # msg.values[0] = MED
    # msg.values[1] = MED
    # msg.values[RC_THR_CHN] = LOW
    # msg.values[3] = MED
    # msg.values[RC_OVD_CHN] = HIGH
    # msg.values[RC_ARM_CHN] = LOW
    # msg.values[6] = MED
    # msg.values[7] = MED
    #
    # loop_counter = 0
    #
    # while not rospy.is_shutdown():
    #     msg.header.stamp = rospy.Time.now()
    #     rc_pub.publish(msg)
    #     rate.sleep()
    #
    #     if loop_counter < calibrate_count:
    #         loop_counter += 1
    #     elif loop_counter == barocal_count:
    #         rospy.loginfo('ATTEMPTING TO CALIBRATE BARO')
    #         subprocess.call(['rosservice','call','/calibrate_baro'])
    #         loop_counter += 1
    #     elif loop_counter > barocal_count and loop_counter < calibrate_count:
    #         loop_counter += 1
    #     elif loop_counter == calibrate_count:
    #         rospy.loginfo('ATTEMPTING TO CALIBRATE IMU')
    #         subprocess.call(['rosservice','call','/calibrate_imu'])
    #         # time.sleep(3.0)
    #         # subprocess.call(['rosservice','call','/calibrate_baro'])
    #         loop_counter += 1
    #     elif loop_counter > calibrate_count and loop_counter < switch_count:
    #         loop_counter += 1
    #     elif loop_counter == switch_count:
    #         rospy.loginfo('SETTING ARM AND OVERRIDE CHANNELS TO HIGH')
    #         msg.values[RC_ARM_CHN] = HIGH
    #         msg.values[RC_OVD_CHN] = LOW
    #         loop_counter += 1
    #     elif loop_counter > switch_count and loop_counter < throttle_count:
    #         loop_counter += 1
    #     elif loop_counter == throttle_count:
    #         rospy.loginfo('SETTING THROTTLE HIGH')
    #         msg.values[RC_THR_CHN] = HIGH
    #         loop_counter += 1
