#!/usr/bin/env python
# -*- coding: cp1252 -*-

#Codigo que envia um valor para o parametro derivativo do controle PID

import rospy
from std_msgs.msg import Float32

pub = rospy.Publisher('kd_set', Float32, queue_size=10)
rospy.init_node('kd_set', anonymous=True)

def set():
    global num
    num = float(input("Digite um valor para kd: "))
    talker()

def talker():
    rospy.loginfo(num) #Mostra o valor no terminal
    pub.publish(num) #Publica o valor no topico kd_set

if __name__ == '__main__':
    while not rospy.is_shutdown():
        set()

