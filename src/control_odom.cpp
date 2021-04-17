#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#define R 0.0325 //Raio da roda em metros
#define len_wheel  0.027 //Largura da roda em metros
#define res_enc  341.2 //341.2 pulsos por revolução
#define L  0.2792 //Distancia entre as rodas em metros
#define L_A  0.22335 //Distancia entre os eixos das rodas frontais e traseiras em metros
#define m_per_tick  (2*3.14159*R)/res_enc //Metros por pulso

float duty_A = 0; //PWM do motor A
float duty_B = 0; //PWM do motor B
float duty_C = 0; //PWM do motor C
float duty_D = 0; //PWM do motor D

float v_A = 0; //Velocidade do motor A
float v_B = 0; //Velocidade do motor B
float v_C = 0; //Velocidade do motor C
float v_D = 0; //Velocidade do motor D

float v_left = 0; //Velocidade dos motores da esquerda
float v_right = 0; //Velocidade dos motores da direita

//Variaveis das velocidades de referencia
float vel_linear_ref = 0; //Velocidade de referencia linear
float vel_angular_ref = 0; //Velocidade de referencia angular
float v_left_ref = 0; //Velocidade de referencia dos motores da esquerda
float v_right_ref = 0; //Velocidade de referencia dos motores da direita

//Variaveis do controle PID
float kp = 335; //Parametro proporcional
float ki = 6.8; //Parametro integral
float kd = 0.0967; //Parametro derivativo

//Variaveis de erro
//Motores da direita
float e_A = 0; //Erro no motor A
float e_P_A = 0; //Erro proporcional no motor A
float e_I_A = 0; //Erro integral no motor A
float e_D_A = 0; //Erro derivativo no motor A
float e_ant_I_A = 0; //Erro integral anterior do motor A
float e_ant_D_A = 0; //Erro derivativo anterior do motor A

float e_D = 0; //Erro no motor D
float e_P_D = 0; //Erro proporcional no motor D
float e_I_D = 0; //Erro integral no motor D
float e_D_D = 0; //Erro derivativo no motor D
float e_ant_I_D = 0; //Erro integral anterior do motor D
float e_ant_D_D = 0; //Erro derivativo anterior do motor D

//Motores da esquerda
float e_B = 0; //Erro no motor B
float e_P_B = 0; //Erro proporcional no motor B
float e_I_B = 0; //Erro integral no motor B
float e_D_B = 0; //Erro derivativo no motor B
float e_ant_I_B = 0; //Erro integral anterior do motor B
float e_ant_D_B = 0; //Erro derivativo anterior do motor B

float e_C = 0; //Erro no motor C
float e_P_C = 0; //Erro proporcional no motor C
float e_I_C = 0; //Erro integral no motor C
float e_D_C = 0; //Erro derivativo no motor C
float e_ant_I_C = 0; //Erro integral anterior do motor C
float e_ant_D_C = 0; //Erro derivativo anterior do motor C
///////////////////////////////////////////////////////////

int hz = 20; //frequencia
float dt_c = 0.05; // resultado de 1/hz
double dt; //periodo

int ticks_A = 0, prev_ticks_A = 0; // Pulsos atuais e anteriores do encoder A
int ticks_B = 0, prev_ticks_B = 0; // Pulsos atuais e anteriores do encoder B
int ticks_C = 0, prev_ticks_C = 0; // Pulsos atuais e anteriores do encoder C
int ticks_D = 0, prev_ticks_D = 0; // Pulsos atuais e anteriores do encoder D

float d_A, d_B, d_C, d_D; //Distancias percorridas pelo motor A, B, C e D
float d_left = 0; //Distancia percorrida pelos motores da esquerda
float d_right = 0; //Distancia percorrida pelos motores da esquerda

double x = 0.0; //Posição em x
double y = 0.0; //Posição em y
double th = 0.0; //Angulo theta

double d_c = 0.0; //Media da distancia percorrida pelos motores dos dois lados do robô
double v = 0.0; //Velocidade do robô

double vx = 0.0; //Velocidade em x
double vy = 0.0; //Velocidade em y
double vth = 0.0; //Velocidade do theta

//Variaveis usadas pelos pubs
std_msgs::Int16 pwm_A_msg;
std_msgs::Int16 pwm_B_msg;
std_msgs::Int16 pwm_C_msg;
std_msgs::Int16 pwm_D_msg;
std_msgs::Float32 vel_A_msg;
std_msgs::Float32 vel_B_msg;
std_msgs::Float32 vel_C_msg;
std_msgs::Float32 vel_D_msg;
std_msgs::Float32 vel_R_ref_msg;
std_msgs::Float32 vel_L_ref_msg;

///////////////////////////////Funções de atualizações dos pulsos////////////////////////
void tickCall_A(const std_msgs::Int64::ConstPtr& msg_A)
{
    ticks_A = msg_A->data;
    //ROS_INFO("I heard: [%i]", ticks_A);
}

void tickCall_B(const std_msgs::Int64::ConstPtr& msg_B)
{
    ticks_B = msg_B->data;
}

void tickCall_C(const std_msgs::Int64::ConstPtr& msg_C)
{
    ticks_C = msg_C->data;
}

void tickCall_D(const std_msgs::Int64::ConstPtr& msg_D)
{
    ticks_D = msg_D->data;
}
//////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////Funções de atualizações dos parametros////////////////////////
void velCall(const geometry_msgs::Twist::ConstPtr& msg_vel)
{
    vel_linear_ref = msg_vel->linear.x;
    vel_angular_ref = msg_vel->angular.z;
    ROS_INFO("Ref linear: [%f] Ref angular: [%f]", vel_linear_ref, vel_angular_ref);
}

void kp_callback(const std_msgs::Float32::ConstPtr& msg_kp)
{
    kp = msg_kp->data;
}

void ki_callback(const std_msgs::Float32::ConstPtr& msg_ki)
{
    ki = msg_ki->data;
}

void kd_callback(const std_msgs::Float32::ConstPtr& msg_kd)
{
    kd = msg_kd->data;
}
//////////////////////////////////////////////////////////////////////////////////////////

void control()
{
    v_right_ref = vel_linear_ref + vel_angular_ref * L;
    v_left_ref = vel_linear_ref - vel_angular_ref * L;

    vel_R_ref_msg.data = v_right_ref;
    vel_L_ref_msg.data = v_left_ref;

    //Motor A
    e_A = v_right_ref - v_A;
    e_P_A = e_A;
    e_I_A = e_ant_I_A + e_A*dt;
    e_D_A = (e_A - e_ant_D_A)/dt;

    if (e_D_A >= 400 || e_D_A <= -400)
    {
        e_ant_D_A = 0;
        e_D_A = 0;
    }

    int(duty_A) = kp*e_P_A + ki*e_I_A + kd*e_D_A;

    if (duty_A > 255)
    {
        duty_A = 255;
    }
    else if (duty_A < -255)
    {
        duty_A = -255;
    }

    //Motor B
    e_B = v_left_ref - v_B;
    e_P_B = e_B;
    e_I_B = e_ant_I_B + e_B*dt;
    e_D_B = (e_B - e_ant_D_B)/dt;

    if (e_D_B >= 400 || e_D_B <= -400)
    {
        e_ant_D_B = 0;
        e_D_B = 0;
    }

    int(duty_B) = kp*e_P_B + ki*e_I_B + kd*e_D_B;

    if (duty_B > 255)
    {
        duty_B = 255;
    }
    else if (duty_B < -255)
    {
        duty_B = -255;
    }

    //Motor C
    e_C = v_left_ref - v_C;
    e_P_C = e_C;
    e_I_C = e_ant_I_C + e_C*dt;
    e_D_C = (e_C - e_ant_D_C)/dt;

    if (e_D_C >= 400 || e_D_C <= -400)
    {
        e_ant_D_C = 0;
        e_D_C = 0;
    }

    int(duty_C) = kp*e_P_C + ki*e_I_C + kd*e_D_C;

    if (duty_C > 255)
    {
        duty_C = 255;
    }
    else if (duty_C < -255)
    {
        duty_C = -255;
    }

    //Motor D
    e_D = v_right_ref - v_D;
    e_P_D = e_D;
    e_I_D = e_ant_I_D + e_D*dt;
    e_D_D = (e_D - e_ant_D_D)/dt;

    if (e_D_D >= 400 || e_D_D <= -400)
    {
        e_ant_D_D = 0;
        e_D_D = 0;
    }   
    
    int(duty_D) = kp*e_P_D + ki*e_I_D + kd*e_D_D;

    if (duty_D > 255)
    {
        duty_D = 255;
    }
    else if (duty_D < -255)
    {
        duty_D = -255;
    }
    
    //ROS_INFO("Parte derivariva esquerda - [%f]", kd*e_D_l);       

    pwm_A_msg.data = duty_A;
    pwm_B_msg.data = duty_B;
    pwm_C_msg.data = duty_C;
    pwm_D_msg.data = duty_D;

    e_ant_I_A = e_I_A;
    e_ant_D_A = e_D_A;

    e_ant_I_B = e_I_B;    
    e_ant_D_B = e_D_B;

    e_ant_I_C = e_I_C;
    e_ant_D_C = e_D_C;

    e_ant_I_D = e_I_D;    
    e_ant_D_D = e_D_D;

    ROS_INFO("Ref lado direito: [%f] Ref lado esquerdo: [%f]", v_right_ref, v_left_ref);
    //ROS_INFO("Lido A: [%f] Lido B: [%f] Lido C: [%f] Lido D: [%f]", v_A, v_B, v_C, v_D);
    //ROS_INFO("Erro A - P[%f] I[%f] D[%f]", e_P_A, e_I_A, e_D_A);
    //ROS_INFO("Erro B - P[%f] I[%f] D[%f]", e_P_B, e_I_B, e_D_B);
    //ROS_INFO("Erro C - P[%f] I[%f] D[%f]", e_P_C, e_I_C, e_D_C);
    //ROS_INFO("Erro D - P[%f] I[%f] D[%f]", e_P_D, e_I_D, e_D_D);
    //ROS_INFO("PWM A: [%d] PWM B: [%d] PWM C: [%d] PWM D: [%d]", duty_A, duty_B, duty_C, duty_D);
}

int main(int argc, char** argv){

  ros::init(argc, argv, "control_odom");
  ros::NodeHandle n;  
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time;
  ros::Time last_time;

/////////////////////////////Declaração do Pubs////////////////////////////////////////

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher pwm_A_pub = n.advertise<std_msgs::Int16>("cmd_vel_A", 50, &pwm_A_msg);
  ros::Publisher pwm_B_pub = n.advertise<std_msgs::Int16>("cmd_vel_B", 50, &pwm_B_msg);
  ros::Publisher pwm_C_pub = n.advertise<std_msgs::Int16>("cmd_vel_C", 50, &pwm_C_msg);
  ros::Publisher pwm_D_pub = n.advertise<std_msgs::Int16>("cmd_vel_D", 50, &pwm_D_msg);
  ros::Publisher vel_A_pub = n.advertise<std_msgs::Float32>("vel_A", 50, &vel_A_msg);
  ros::Publisher vel_B_pub = n.advertise<std_msgs::Float32>("vel_B", 50, &vel_B_msg);
  ros::Publisher vel_C_pub = n.advertise<std_msgs::Float32>("vel_C", 50, &vel_C_msg);
  ros::Publisher vel_D_pub = n.advertise<std_msgs::Float32>("vel_D", 50, &vel_D_msg);
  ros::Publisher vel_R_ref_pub = n.advertise<std_msgs::Float32>("vel_R_ref", 50, &vel_R_ref_msg);
  ros::Publisher vel_L_ref_pub = n.advertise<std_msgs::Float32>("vel_L_ref", 50, &vel_L_ref_msg);

////////////////////////////Declaração dos Subs/////////////////////////////////////////

  ros::Subscriber tick_A = n.subscribe<std_msgs::Int64>("ticks_A", 1000, tickCall_A);
  ros::Subscriber tick_B = n.subscribe<std_msgs::Int64>("ticks_B", 1000, tickCall_B);
  ros::Subscriber tick_C = n.subscribe<std_msgs::Int64>("ticks_C", 1000, tickCall_C);
  ros::Subscriber tick_D = n.subscribe<std_msgs::Int64>("ticks_D", 1000, tickCall_D);
  ros::Subscriber vel = n.subscribe<geometry_msgs::Twist>("cmd_vel",1000, velCall);
  ros::Subscriber set_kp = n.subscribe<std_msgs::Float32>("kp_set",1000, kp_callback);
  ros::Subscriber set_ki = n.subscribe<std_msgs::Float32>("ki_set",1000, ki_callback);
  ros::Subscriber set_kd = n.subscribe<std_msgs::Float32>("kd_set",1000, kd_callback);


  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(hz);

  while(n.ok()){

    ros::spinOnce(); //Verifica se há mensagens recebidas

    current_time = ros::Time::now(); //Atualização da variavel de tempo

    dt = (current_time - last_time).toSec(); // Atualização do periodo

    ROS_INFO("dt[%f]", dt);

    d_A = m_per_tick*(ticks_A - prev_ticks_A); //Distancia percorrida pelo motor A
    d_B = m_per_tick*(ticks_B - prev_ticks_B); //Distancia percorrida pelo motor B
    d_C = m_per_tick*(ticks_C - prev_ticks_C); //Distancia percorrida pelo motor C
    d_D = m_per_tick*(ticks_D - prev_ticks_D); //Distancia percorrida pelo motor D

    v_A = d_A/dt; //Velocidade do motor A
    v_B = d_B/dt; //Velocidade do motor B
    v_C = d_C/dt; //Velocidade do motor C
    v_D = d_D/dt; //Velocidade do motor D

    vel_A_msg.data = v_A; //Velocidade do motor A
    vel_B_msg.data = v_B; //Velocidade do motor B
    vel_C_msg.data = v_C; //Velocidade do motor C
    vel_D_msg.data = v_D; //Velocidade do motor D

    prev_ticks_A = ticks_A; //Atualizaçao dos pulsos passados do encoder A
    prev_ticks_B = ticks_B; //Atualizaçao dos pulsos passados do encoder B
    prev_ticks_C = ticks_C; //Atualizaçao dos pulsos passados do encoder C
    prev_ticks_D = ticks_D; //Atualizaçao dos pulsos passados do encoder D

    d_left = (d_B + d_C)/2; //Media da distancia percorrida dos motores da esquerda em metros
    d_right = (d_A + d_D)/2; //Media da distancia percorrida dos motores da direita em metros
    d_c = (d_left + d_right)/2; //Media da distancia percorrida dos dois lados

    //ROS_INFO("d_c:[%f], d_left:[%f], d_right:[%f]", d_c, d_left, d_right);

    v_left = d_left/dt; //Velocidade dos motores da esquerda
    v_right = d_right/dt; //Velocidade dos motores da direita

    vth = ((d_right-d_left )/L)/dt; //Velocidade angular

    vx = (cos(th) * d_c)/dt; //Velocidade em x
    vy = (sin(th) * d_c)/dt; //Velocidade em y

    //ROS_INFO("Vx:[%f], Vy:[%f], Vth:[%f]", vx, vy, vth);

    //Calculo da odometria de uma maneira tipica, dadas as velocidades do robô
    double delta_x = ((vx * cos(th)) - (vy * sin(th)))* dt;
    double delta_y = ((vx * sin(th)) + (vy * cos(th)))* dt;
    //double delta_x = vx * dt;
    //double delta_y = vy * dt;
    double delta_th = vth * dt;
    //ROS_INFO("delta_x:[%f], delta_y:[%f], delta_th:[%f]", delta_x, delta_y, delta_th);

    x += delta_x; //Atualização da posição em x
    y += delta_y; //Atualização da posição em y
    th += delta_th; //Atualização do angulo theta
    //ROS_INFO("x:[%f], y:[%f], th:[%f]", x, y, th);

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

    //Publicação da transformação de tf
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

    //Mensagens da Odometria
    nav_msgs::Odometry odom;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //Definição da posição
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //Definição da velocidade
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = vth;

    control();
    
    //Publicação dos valores de PWM de cada motor
    pwm_A_pub.publish( pwm_A_msg);
    pwm_B_pub.publish( pwm_B_msg);
    pwm_C_pub.publish( pwm_C_msg);
    pwm_D_pub.publish( pwm_D_msg);

    //Publicação das velocidades
    vel_R_ref_pub.publish( vel_R_ref_msg);
    vel_L_ref_pub.publish( vel_L_ref_msg);
    vel_A_pub.publish( vel_A_msg);
    vel_B_pub.publish( vel_B_msg);
    vel_C_pub.publish( vel_C_msg);
    vel_D_pub.publish( vel_D_msg);

    last_time = current_time; //Atualização da variavel do tempo passado

    //Envio da transformação
    odom_broadcaster.sendTransform(odom_trans);
    //Publicação da odometria
    odom_pub.publish(odom); 

    r.sleep();
  }
}
