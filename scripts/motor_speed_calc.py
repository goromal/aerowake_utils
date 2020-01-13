#!/usr/bin/python
import rospy
from rosflight_msgs.msg import OutputRaw
from mav_msgs.msg import Actuators

class MotorSpeedCalculator(object):
    def __init__(self):
        self.signalSub = rospy.Subscriber('signals', OutputRaw, self.signalCallback)
        self.speedPub = rospy.Publisher('motor_speeds', Actuators, queue_size=1)
        self.kop = rospy.get_param('~motor/k_omega_poly')

    def signalCallback(self, msg):
        actuators_msg = Actuators()
        actuators_msg.header = msg.header
        for i in range(4):
            s = msg.values[i]
            w = self.kop[0] * s + self.kop[1] # calculate motor speed
            actuators_msg.angular_velocities.append(w)
        self.speedPub.publish(actuators_msg)

if __name__ == '__main__':
    rospy.init_node('motor_speed_calculator', anonymous=True)
    MSC = MotorSpeedCalculator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down motor speed calculator node.'
