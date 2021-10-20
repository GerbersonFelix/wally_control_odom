#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


def TA_callback(msg_TA):
    global ticks_A
    ticks_A = msg_TA.data

def TB_callback(msg_TB):
    global ticks_B
    ticks_B = msg_TB.data

def TC_callback(msg_TC):
    global ticks_C
    ticks_C = msg_TC.data

def TD_callback(msg_TD):
    global ticks_D
    ticks_D = msg_TD.data

rospy.init_node('odometry_publisher')

odom_broadcaster = tf.TransformBroadcaster()
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
rospy.Subscriber("ticks_A", Int64, TA_callback)
rospy.Subscriber("ticks_B", Int64, TB_callback)
rospy.Subscriber("ticks_C", Int64, TC_callback)
rospy.Subscriber("ticks_D", Int64, TD_callback)

x = 0.0
y = 0.0
th = 0.0

vx = 0.1
vy = -0.1
vth = 0.1

ticks_A = 0
prev_ticks_A = 0 # Pulsos atuais e anteriores do encoder A
ticks_B = 0
prev_ticks_B = 0 # Pulsos atuais e anteriores do encoder B
ticks_C = 0
prev_ticks_C = 0 # Pulsos atuais e anteriores do encoder C
ticks_D = 0
prev_ticks_D = 0 # Pulsos atuais e anteriores do encoder D

d = 0
d_A = 0
d_B = 0
d_C = 0
d_D = 0
d_left = 0
d_right = 0

v = 0
v_left = 0
v_right = 0
v_A = 0 #Velocidade do motor A
v_B = 0 #Velocidade do motor B
v_C = 0 #Velocidade do motor C
v_D = 0 #Velocidade do motor D

vel_linear_ref = 0 #Velocidade de referencia linear
vel_angular_ref = 0 #Velocidade de referencia angular
v_left_ref = 0 #Velocidade de referencia dos motores da esquerda
v_right_ref = 0 #Velocidade de referencia dos motores da direita

hz = 20 #Frequencia
R = 0.0325 #Raio da roda em metros
len_wheel = 0.027 #Largura da roda em metros
res_enc = 341.2 #Pulsos por revolucao
L = 0.2792 #Distancia entre as rodas em metros
W = 0.22335 #Distancia entre os eixos das rodas frontais e traseiras em metros
m_per_tick = (2*pi*R)/res_enc #Metros por pulso

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(hz)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

    d_A = m_per_tick*(ticks_A - prev_ticks_A) #Distancia percorrida pelo motor A
    d_B = m_per_tick*(ticks_B - prev_ticks_B) #Distancia percorrida pelo motor B
    d_C = m_per_tick*(ticks_C - prev_ticks_C) #Distancia percorrida pelo motor C
    d_D = m_per_tick*(ticks_D - prev_ticks_D) #Distancia percorrida pelo motor D

    prev_ticks_A = ticks_A #Atualizacao dos pulsos passados do encoder A
    prev_ticks_B = ticks_B #Atualizacao dos pulsos passados do encoder B
    prev_ticks_C = ticks_C #Atualizacao dos pulsos passados do encoder C
    prev_ticks_D = ticks_D #Atualizacao dos pulsos passados do encoder D

    d_left = (d_B + d_C)/2 #Media da distancia percorrida dos motores da esquerda
    d_right = (d_A + d_D)/2 #Media da distancia percorrida dos motores da direita
    d = (d_left + d_right)/2 #Media da distancia percorrida dos dois lados

    v_left = d_left/dt #Velocidade das rodas da direita
    v_right = d_right/dt #Velocidade das rodas da esquerda
    v = d/dt #Velocidade do robo

    vx = v
    vy = 0
    vth = (v_right-v_left)/L

    # compute odometry in a typical way given the velocities of the robot    
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()