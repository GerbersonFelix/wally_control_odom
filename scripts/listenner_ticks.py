#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64

def ticks_callback(msg_ticks):
    global ticks
    global encoder_A
    global encoder_B
    global encoder_C
    global encoder_D
    global l_wheels
    global r_wheels



    ticks = msg_ticks.data
    encoders = ticks.split('#')
    rospy.loginfo('Encoders: %s', encoders)
    encoder_A = int(encoders[0])
    encoder_B = int(encoders[1])
    encoder_C = int(encoders[2])
    encoder_D = int(encoders[3])
    l_wheels = (encoder_B + encoder_C)/2
    r_wheels = (encoder_A + encoder_D)/2
    rospy.loginfo('A: %d B: %d C: %d D: %d LW: %f RW: %f', encoder_A, encoder_B, encoder_C, encoder_D, l_wheels, r_wheels)

    talker()


def set():
    global pub_A
    global pub_B
    global pub_C
    global pub_D
    global pub_L
    global pub_R
    
    rospy.init_node('listenner_ticks', anonymous=True)
    rospy.Subscriber("ticks", String, ticks_callback)
    pub_A = rospy.Publisher('ticks_A', Int64, queue_size=10)
    pub_B = rospy.Publisher('ticks_B', Int64, queue_size=10)
    pub_C = rospy.Publisher('ticks_C', Int64, queue_size=10)
    pub_D = rospy.Publisher('ticks_D', Int64, queue_size=10)
    pub_L = rospy.Publisher('lwheels', Int64, queue_size=10)
    pub_R = rospy.Publisher('rwheels', Int64, queue_size=10)

    rospy.spin()




def talker():


    #rospy.loginfo('Encoder_A: %d', encoder_A)
    pub_A.publish(encoder_A)

    #rospy.loginfo('Encoder_B: %d', encoder_B)
    pub_B.publish(encoder_B)

    #rospy.loginfo('Encoder_C: %d', encoder_C)
    pub_C.publish(encoder_C)

    #rospy.loginfo('Encoder_D: %d', encoder_D)
    pub_D.publish(encoder_D)

    #rospy.loginfo('LW: %f', l_wheels)
    pub_L.publish(l_wheels)

    #rospy.loginfo('RW: %f', r_wheels)
    pub_R.publish(r_wheels)



if __name__ == '__main__':
    while not rospy.is_shutdown():
        set()
