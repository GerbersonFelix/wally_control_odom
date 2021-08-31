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

int duty_A = 0; //PWM do motor A
int duty_B = 0; //PWM do motor B
int duty_C = 0; //PWM do motor C
int duty_D = 0; //PWM do motor D

//Variaveis das velocidades de referencia
float vel_linear_ref = 0; //Velocidade de referencia linear
float vel_angular_ref = 0; //Velocidade de referencia angular
float v_left_ref = 0; //Velocidade de referencia dos motores da esquerda
float v_right_ref = 0; //Velocidade de referencia dos motores da direita

int hz = 20; //frequencia
double dt; //periodo

int ticks_A = 0, prev_ticks_A = 0; // Pulsos atuais e anteriores do encoder A
int ticks_B = 0, prev_ticks_B = 0; // Pulsos atuais e anteriores do encoder B
int ticks_C = 0, prev_ticks_C = 0; // Pulsos atuais e anteriores do encoder C
int ticks_D = 0, prev_ticks_D = 0; // Pulsos atuais e anteriores do encoder D

float d = 0; //Distancia percorrida pelo robô
float d_A = 0; //Distancia percorrida pelo motor A
float d_B = 0; //Distancia percorrida pelo motor B
float d_C = 0; //Distancia percorrida pelo motor C
float d_D = 0; //Distancia percorrida pelo motor D
float d_left = 0; //Distancia percorrida pelos motores da esquerda
float d_right = 0; //Distancia percorrida pelos motores da direita

double v = 0.0; //Velocidade do robô
float v_left = 0; //Velocidade dos motores da esquerda
float v_right = 0; //Velocidade dos motores da direita
float v_A = 0; //Velocidade do motor A
float v_B = 0; //Velocidade do motor B
float v_C = 0; //Velocidade do motor C
float v_D = 0; //Velocidade do motor D

double x = 0.0; //Posição em x
double y = 0.0; //Posição em y
double th = 0.0; //Angulo theta

double vx = 0.0; //Velocidade em x
double vy = 0.0; //Velocidade em y
double vth = 0.0; //Velocidade do theta

//Variaveis do controle PID
float kp = 120; //Parametro proporcional
float ki = 30; //Parametro integral
float kd = 60; //Parametro derivativo

int pwmA_min = -255, pwmA_max = 255;
int pwmB_min = -255, pwmB_max = 255;
int pwmC_min = -255, pwmC_max = 255;
int pwmD_min = -255, pwmD_max = 255;

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

int map(int x_f, int in_min, int in_max, int out_min, int out_max){
    return int((x_f-in_min) * (out_max-out_min) / (in_max-in_min) + out_min);
}

int val_min(int atual, int antigo){
  if (atual < antigo){
    return atual;
  }        
  else{
    return antigo;
  }        
}   

int val_max(int atual, int antigo)
{
    if (atual > antigo){
      return atual;
    }        
    else{
        return antigo;
    }
}
void velCall(const geometry_msgs::Twist::ConstPtr& msg_vel)
{
    vel_linear_ref = msg_vel->linear.x;
    vel_angular_ref = msg_vel->angular.z;
    //ROS_INFO("Ref linear: [%f] Ref angular: [%f]", vel_linear_ref, vel_angular_ref);

    v_right_ref = vel_linear_ref + vel_angular_ref * L;
    v_left_ref = vel_linear_ref - vel_angular_ref * L;

    vel_R_ref_msg.data = v_right_ref;
    vel_L_ref_msg.data = v_left_ref;
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
    e_A = v_right_ref - v_A; //Calculo do erro do motor A
    e_P_A = e_A;
    e_I_A = e_ant_I_A + e_A;
    e_D_A = e_ant_D_A;

    e_B = v_left_ref - v_B; //Calculo do erro do motor B
    e_P_B = e_B;
    e_I_B = e_ant_I_B + e_B;
    e_D_B = e_ant_D_B;

    e_C = v_left_ref - v_C; //Calculo do erro do motor C
    e_P_C = e_C;
    e_I_C = e_ant_I_C + e_C;
    e_D_C = e_ant_D_C;

    e_D = v_right_ref - v_D; //Calculo do erro do motor D
    e_P_D = e_D;
    e_I_D = e_ant_I_D + e_D;
    e_D_D = e_ant_D_D;

    duty_A = kp*e_P_A + ki*e_I_A + kd*e_D_A;
    duty_B = kp*e_P_B + ki*e_I_B + kd*e_D_B;
    duty_C = kp*e_P_C + ki*e_I_C + kd*e_D_C;
    duty_D = kp*e_P_D + ki*e_I_D + kd*e_D_D;

    pwmA_min = val_min(duty_A,pwmA_min);
    pwmA_max = val_max(duty_A,pwmA_max);

    pwmB_min = val_min(duty_B,pwmB_min);
    pwmB_max = val_max(duty_B,pwmB_max);

    pwmC_min = val_min(duty_C,pwmC_min);
    pwmC_max = val_max(duty_C,pwmC_max);

    pwmD_min = val_min(duty_D,pwmD_min);
    pwmD_max = val_max(duty_D,pwmD_max);

    pwm_A_msg.data = map(duty_A,pwmA_min,pwmA_max,-255,255);
    pwm_B_msg.data = map(duty_B,pwmB_min,pwmB_max,-255,255);
    pwm_C_msg.data = map(duty_C,pwmC_min,pwmC_max,-255,255);
    pwm_D_msg.data = map(duty_D,pwmD_min,pwmD_max,-255,255);

    ROS_INFO("PWM A: [%d] PWM B: [%d] PWM C: [%d] PWM D: [%d]", pwm_A_msg.data, pwm_B_msg.data, pwm_C_msg.data, pwm_D_msg.data);

    e_ant_I_A = e_I_A;
    e_ant_D_A = e_A;

    e_ant_I_B = e_I_B; 
    e_ant_D_B = e_B;

    e_ant_I_C = e_I_C;
    e_ant_D_C = e_C;

    e_ant_I_D = e_I_D; 
    e_ant_D_D = e_D;

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

    dt = (current_time - last_time).toSec(); //Atualização do periodo

    //ROS_INFO("dt[%f]", dt);

    d_A = m_per_tick*(ticks_A - prev_ticks_A); //Distancia percorrida pelo motor A
    d_B = m_per_tick*(ticks_B - prev_ticks_B); //Distancia percorrida pelo motor B
    d_C = m_per_tick*(ticks_C - prev_ticks_C); //Distancia percorrida pelo motor C
    d_D = m_per_tick*(ticks_D - prev_ticks_D); //Distancia percorrida pelo motor D

    v_A = d_A/dt; //Velocidade do motor A em m/s
    v_B = d_B/dt; //Velocidade do motor B em m/s
    v_C = d_C/dt; //Velocidade do motor C em m/s
    v_D = d_D/dt; //Velocidade do motor D em m/s

    vel_A_msg.data = v_A; //Velocidade do motor A em m/s
    vel_B_msg.data = v_B; //Velocidade do motor B em m/s
    vel_C_msg.data = v_C; //Velocidade do motor C em m/s
    vel_D_msg.data = v_D; //Velocidade do motor D em m/s

    prev_ticks_A = ticks_A; //Atualizaçao dos pulsos passados do encoder A
    prev_ticks_B = ticks_B; //Atualizaçao dos pulsos passados do encoder B
    prev_ticks_C = ticks_C; //Atualizaçao dos pulsos passados do encoder C
    prev_ticks_D = ticks_D; //Atualizaçao dos pulsos passados do encoder D

    d_left = (d_B + d_C)/2; //Media da distancia percorrida dos motores da esquerda em metros
    d_right = (d_A + d_D)/2; //Media da distancia percorrida dos motores da direita em metros
    d = (d_left + d_right)/2; //Media da distancia percorrida dos dois lados
    ROS_INFO("d:[%f], d_left:[%f], d_right:[%f]", d, d_left, d_right);

    v_left = d_left/dt; //Velocidade das rodas da direita
    v_right = d_right/dt; //Velocidade das rodas da esquerda
    v = d/dt; //Velocidade do robô

    vx = v;
    vy = 0;
    vth = (v_right-v_left)/L;
    ROS_INFO("Vx:[%f], Vy:[%f], Vth:[%f]", vx, vy, vth);

    //Calculo da odometria de uma maneira tipica, dadas as velocidades do robô
    double delta_x = ((vx * cos(th)) - (vy * sin(th)))* dt;
    double delta_y = ((vx * sin(th)) + (vy * cos(th)))* dt;
    double delta_th = vth * dt;
    ROS_INFO("delta_x:[%f], delta_y:[%f], delta_th:[%f]", delta_x, delta_y, delta_th);

    x += delta_x; //Atualização da posição em x
    y += delta_y; //Atualização da posição em y
    th += delta_th; //Atualização do angulo theta
    ROS_INFO("x:[%f], y:[%f], th:[%f]", x, y, th);

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0 ,0, th);

    //Publicação da transformação de tf
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0543676401358; //Altura do base_link em relação ao solo
    odom_trans.transform.rotation = odom_quat;

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

    ROS_INFO("Ref lado direito: [%f] Ref lado esquerdo: [%f]", v_right_ref, v_left_ref);
    ROS_INFO("Lido A: [%f] Lido B: [%f] Lido C: [%f] Lido D: [%f]", v_A, v_B, v_C, v_D);

    last_time = current_time; //Atualização da variavel do tempo passado

    //Envio da transformação
    odom_broadcaster.sendTransform(odom_trans);
    //Publicação da odometria
    odom_pub.publish(odom); 

    r.sleep();







    
  }
}
