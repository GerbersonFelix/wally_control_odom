#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

float R = 0.0325; //Raio da roda em metros
float len_wheel = 0.027; //Largura da roda em metros
float res_enc = 341.2; //341.2 pulsos por revolução
float L = 0.2792; //Distancia entre as rodas em metros
float L_A = 0.22335; //Distancia entre os eixos das rodas frontais e traseiras em metros
float m_per_tick = (2*3.14159*R)/res_enc; //Metros por pulso

int ticks_A = 0, prev_ticks_A = 0; // Pulsos atuais e anteriores do encoder A
int ticks_B = 0, prev_ticks_B = 0; // Pulsos atuais e anteriores do encoder B
int ticks_C = 0, prev_ticks_C = 0; // Pulsos atuais e anteriores do encoder C
int ticks_D = 0, prev_ticks_D = 0; // Pulsos atuais e anteriores do encoder D

float d = 0; //Distancias percorridas pelo robô
float d_A = 0; //Distancias percorridas pelo motor A
float d_B = 0; //Distancias percorridas pelo motor B
float d_C = 0; //Distancias percorridas pelo motor C
float d_D = 0; //Distancias percorridas pelo motor D
float d_left = 0; //Distancia percorrida pelos motores da esquerda
float d_right = 0; //Distancia percorrida pelos motores da direita

double x = 0.0; //Posição em x
double y = 0.0; //Posição em y
double th = 0.0; //Angulo theta

int hz = 20; //Frequencia
double dt; //Periodo

double vx = 0.1; //Velocidade em x
double vy = -0.1; //Velocidade em y
double vth = 0.1; //Velocidade angular

float v = 0; //Velocidade do robô
float v_left = 0; //Velocidade dos motores da esquerda
float v_right = 0; //Velocidade dos motores da direita

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

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time;
  ros::Time last_time;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber tick_A = n.subscribe<std_msgs::Int64>("ticks_A", 1000, tickCall_A);
  ros::Subscriber tick_B = n.subscribe<std_msgs::Int64>("ticks_B", 1000, tickCall_B);
  ros::Subscriber tick_C = n.subscribe<std_msgs::Int64>("ticks_C", 1000, tickCall_C);
  ros::Subscriber tick_D = n.subscribe<std_msgs::Int64>("ticks_D", 1000, tickCall_D);
  
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(hz);

  while(n.ok()){

    ros::spinOnce(); //Verifica se há mensagens recebidas

    current_time = ros::Time::now(); //Atualização da variavel de tempo

    dt = (current_time - last_time).toSec(); //Atualização do periodo

    d_A = m_per_tick*(ticks_A - prev_ticks_A); //Distancia percorrida pelo motor A
    d_B = m_per_tick*(ticks_B - prev_ticks_B); //Distancia percorrida pelo motor B
    d_C = m_per_tick*(ticks_C - prev_ticks_C); //Distancia percorrida pelo motor C
    d_D = m_per_tick*(ticks_D - prev_ticks_D); //Distancia percorrida pelo motor D    

    prev_ticks_A = ticks_A; //Atualizaçao dos pulsos passados do encoder A
    prev_ticks_B = ticks_B; //Atualizaçao dos pulsos passados do encoder B
    prev_ticks_C = ticks_C; //Atualizaçao dos pulsos passados do encoder C
    prev_ticks_D = ticks_D; //Atualizaçao dos pulsos passados do encoder D
    
    d_left = (d_B + d_C)/2; //Media da distancia percorrida dos motores da esquerda
    d_right = (d_A + d_D)/2; //Media da distancia percorrida dos motores da direita
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
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, th);
    
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

    last_time = current_time;

    //Envio da transformação
    odom_broadcaster.sendTransform(odom_trans);
    //Publicação da odometria
    odom_pub.publish(odom); 

    r.sleep();
  }
}



