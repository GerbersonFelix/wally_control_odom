/*
Especificações:
– Tensão: 6V DC
– Velocidade: 210 RPM
– Diâmetro do eixo: 4mm
– Comprimento do eixo: 12mm
– Máxima eficiência: 2.0kg.cm/170rpm/2.0W/0.60A
– Máximo torque: 5.2kg.cm/110rpm/3.1W/1.10A
– Cabo de conexão: 6 fios
– Comprimento do cabo: 20 cm
– Comprimento total motor e eixo: 73mm
- Redução  1:34 (1:34.02)
- Resolução do Encoder  11 x Redução 34.02 = 341.2PPR
- Diâmetro do motor 25mm
– Largura da roda: 27mm
– Diâmetro da roda: 65mm

Pinagem:
– 1 – M1 Motor (Vermelho)
– 2 – GND Encoder (Preto)
– 3 – Encoder A phase (Amarelo)
– 4 – Encoder B phase (Verde)
– 5 – 3.3V Encoder (Azul)
– 6 – M1 motor (Branco)
*/

#define USE_USBCON// Para usar com o arduino Due para não dar o erro "Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino"
//#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/Joy.h> //Para Joystick
#include <std_msgs/Int16.h> //Para PWM
#include <std_msgs/String.h> //Para encoders

//////////////////////////////////////ROS///////////////////////////////////////
ros::NodeHandle nh;

std_msgs::String ticks_msg;
ros::Publisher ticks("ticks", &ticks_msg);

/*Pinagem do arduino*/
//////Ponte01///////
//motor_A
const int IN1 = 12;
const int IN2 = 11;
const int velocidadeA = 13;

//motor_B
const int IN3 = 10;
const int IN4 = 9;
const int velocidadeB = 8;

///////Ponte02/////
//motor_C
const int IN5 = 3;
const int IN6 = 4;
const int velocidadeC = 2;

//motor_D
const int IN7 = 5;
const int IN8 = 6;
const int velocidadeD = 7;

/////////Encoders////////
//Encoder motor A
const int C1 = 34;
const int C2 = 35;
//Encoder motor B
const int C3 = 36;
const int C4 = 37;
//Encoder motor C
const int C5 = 30;
const int C6 = 31;
//Encoder motor D
const int C7 = 32;
const int C8 = 33;

char motores[80];
//Motor A(Lado direito)
volatile int long temp_A, counter_A;
char motor_a[10];
//Motor B(Lado esquerdo)
volatile int long temp_B, counter_B;
char motor_b[10];
//Motor C(Lado esquerdo)
volatile int long temp_C, counter_C;
char motor_c[10];
//Motor D(Lado direito)
volatile int long temp_D, counter_D;
char motor_d[10];
//amarelo C1
//verde C2

///////////////////////////////Variaveis usadas////////////////////////////////
int vel = 30; //Velocidade do prototipo
bool cmd_joy = false;
char vel_on[80];
unsigned long range_timer = 0;
int t_pub = 50; //tempo de pub em milisegundos; Frequencia de 20Hz
int ativo;
int cmd_vel_A, cmd_vel_B, cmd_vel_C, cmd_vel_D;
int cmd_vel_RT, cmd_vel_LT, cmd_vel_ANALOG_LEFT;

//Botões do joystick
int A = 0, B = 1, X = 3, Y = 4;//No controle de XBOX A = 0, B = 1, X = 2, Y = 3;
int LB = 6, RB = 7, BACK = 10, START = 11;//No controle de XBOX LB = 4, RB = 5, BACK = 6, START = 7, XBOX = 8;
//Eixos do joystick
int LEFT_ANALOG_UP_DOWN = 1, LEFT_ANALOG_RIGHT_LEFT = 0, RIGHT_ANALOG_UP_DOWN = 3, RIGHT_ANALOG_RIGHT_LEFT = 2, DPAD_UP_DOWN = 7, DPAD_RIGHT_LEFT = 6, RT = 4, LT = 5;

///////////////////////Funções de interrupção dos encoders/////////////////////
void ai0()
{
  if (digitalRead(C2) == LOW)
  {
    counter_A++;
  }
}

void ai1()
{
  if (digitalRead(C1) == LOW)
  {
    counter_A--;
  }
}

void ai2()
{
  if (digitalRead(C4) == LOW)
  {
    counter_B--;
  }
}

void ai3()
{
  if (digitalRead(C3) == LOW)
  {
    counter_B++;
  }
}

void ai4()
{
  if (digitalRead(C6) == LOW)
  {
    counter_C--;
  }
}

void ai5()
{
  if (digitalRead(C5) == LOW)
  {
    counter_C++;
  }
}

void ai6()
{
  if (digitalRead(C8) == LOW)
  {
    counter_D++;
  }
}

void ai7()
{
  if (digitalRead(C7) == LOW)
  {
    counter_D--;
  }
}

////////////////////////Funções de avanço dos motores//////////////////
void go_A(int duty_A)
{
    //Motor A
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(velocidadeA,duty_A);
}

void go_B(int duty_B)
{
    //Motor B
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    analogWrite(velocidadeB,duty_B);
}

void go_C(int duty_C)
{
    //Motor C
    digitalWrite(IN5,LOW);
    digitalWrite(IN6,HIGH);
    analogWrite(velocidadeC,duty_C);
}

void go_D(int duty_D)
{
    //Motor D
    digitalWrite(IN7,LOW);
    digitalWrite(IN8,HIGH);
    analogWrite(velocidadeD,duty_D);
}

///////////////Funções de recuo dos motores/////////////
void back_A(int duty_A)
{
    //Motor A
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(velocidadeA,duty_A);
}

void back_B(int duty_B)
{
    //Motor B
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(velocidadeB,duty_B);
}

void back_C(int duty_C)
{
    //Motor C
    digitalWrite(IN5,HIGH);
    digitalWrite(IN6,LOW);
    analogWrite(velocidadeC,duty_C);
}

void back_D(int duty_D)
{
    //Motor D
    digitalWrite(IN7,HIGH);
    digitalWrite(IN8,LOW);
    analogWrite(velocidadeD,duty_D);
}

/////////////////////Funções pre-definidas//////////////////

void go(int duty_A,int duty_B,int duty_C,int duty_D)
{
    //Motores A
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    //Motores B
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    analogWrite(velocidadeA,duty_A);
    analogWrite(velocidadeB,duty_B);
    //Motores C
    digitalWrite(IN5,LOW);
    digitalWrite(IN6,HIGH);
    //Motores D
    digitalWrite(IN7,LOW);
    digitalWrite(IN8,HIGH);
    analogWrite(velocidadeC,duty_C);
    analogWrite(velocidadeD,duty_D);
}

void back(int duty_A,int duty_B,int duty_C,int duty_D)
{
    //Motores A
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    //Motores B
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(velocidadeA,duty_A);
    analogWrite(velocidadeB,duty_B);
    //Motores C
    digitalWrite(IN5,HIGH);
    digitalWrite(IN6,LOW);
    //Motores D
    digitalWrite(IN7,HIGH);
    digitalWrite(IN8,LOW);
    analogWrite(velocidadeC,duty_C);
    analogWrite(velocidadeD,duty_D);
}

void left(int duty_A,int duty_B,int duty_C,int duty_D)
{
    //Motores A
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    //Motores B
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(velocidadeA,duty_A);
    analogWrite(velocidadeB,duty_B);
    //Motores C
    digitalWrite(IN5,HIGH);
    digitalWrite(IN6,LOW);
    //Motores D
    digitalWrite(IN7,LOW);
    digitalWrite(IN8,HIGH);
    analogWrite(velocidadeC,duty_C);
    analogWrite(velocidadeD,duty_D);
}

void right(int duty_A,int duty_B,int duty_C,int duty_D)
{
    //Motores A
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    //Motores B
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    analogWrite(velocidadeA,duty_A);
    analogWrite(velocidadeB,duty_B);
    //Motores C
    digitalWrite(IN5,LOW);
    digitalWrite(IN6,HIGH);
    //Motores D
    digitalWrite(IN7,HIGH);
    digitalWrite(IN8,LOW);
    analogWrite(velocidadeC,duty_C);
    analogWrite(velocidadeD,duty_D);
}

void stop_motor()
{
    //Motores A
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    //Motores B
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
    analogWrite(velocidadeA,0);
    analogWrite(velocidadeB,0);
    //Motores C
    digitalWrite(IN5,LOW);
    digitalWrite(IN6,LOW);
    //Motores D
    digitalWrite(IN7,LOW);
    digitalWrite(IN8,LOW);
    analogWrite(velocidadeC,0);
    analogWrite(velocidadeD,0);
}

/////////////////Funções de CallBack para controle da velocidade dos motores/////////////
void velCallBack_A(const std_msgs::Int16& msg_A) //Função de comandos recebidos da Jetson
{
  cmd_vel_A = msg_A.data;
  if(cmd_joy == false){
    if(cmd_vel_A>=0){
    go_A(cmd_vel_A);
    }
    else{
      back_A(cmd_vel_A*-1);
    }
  }
}

void velCallBack_B(const std_msgs::Int16& msg_B) //Função de comandos recebidos da Jetson
{
  cmd_vel_B = msg_B.data;
  if(cmd_joy == false){
    if(cmd_vel_B>=0){
    go_B(cmd_vel_B);
    }
    else{
      back_B(cmd_vel_B*-1);
    }
  }
}

void velCallBack_C(const std_msgs::Int16& msg_C) //Função de comandos recebidos da Jetson
{
  cmd_vel_C = msg_C.data;
  if(cmd_joy == false){
    if(cmd_vel_C>=0){
    go_C(cmd_vel_C);
    }
    else{
      back_C(cmd_vel_C*-1);
    }
  }
}

void velCallBack_D(const std_msgs::Int16& msg_D) //Função de comandos recebidos da Jetson
{
  cmd_vel_D = msg_D.data;
  if(cmd_joy == false){
    if(cmd_vel_D>=0){
    go_D(cmd_vel_D);
    }
    else{
      back_D(cmd_vel_D*-1);
    }
  }
}

void joyCallBack(const sensor_msgs::Joy& joy) //Função de comando recebidos do joystick
{
  
  if(joy.buttons[A] == 1)
  {
    stop_motor();
  }
  if(joy.buttons[B] == 1)
  {
    stop_motor();
  }
  if(joy.buttons[X] == 1)
  {
    stop_motor();
  }
  if(joy.buttons[Y] == 1)
  {
    stop_motor();
  }
  
  if(joy.buttons[BACK] == 1)
  {
    cmd_joy = !cmd_joy;
    if(cmd_joy == true){
      nh.loginfo("Joy ativado");
    }
    else{
      nh.loginfo("Joy desativado");
    }
  }
  if(joy.buttons[START] == 1)
  {
    stop_motor();
  }

  if(cmd_joy == true){
    if(joy.axes[RT] != 1)
    {
      cmd_vel_RT = joy.axes[RT]*-100;
      cmd_vel_RT = map(cmd_vel_RT, -100, 99, 0, 180);
      go(cmd_vel_RT,cmd_vel_RT,cmd_vel_RT,cmd_vel_RT);
    }
    if(joy.axes[LT] != 1)
    {
      cmd_vel_LT = joy.axes[LT]*-100;
      cmd_vel_LT = map(cmd_vel_LT, -100, 99, 0, 180);
      back(cmd_vel_LT,cmd_vel_LT,cmd_vel_LT,cmd_vel_LT);
    }

    if(joy.axes[LEFT_ANALOG_RIGHT_LEFT] > 0)
    {
      cmd_vel_ANALOG_LEFT = joy.axes[LEFT_ANALOG_RIGHT_LEFT]*100;
      cmd_vel_ANALOG_LEFT = map(cmd_vel_ANALOG_LEFT, 0, 99, 0, 180);
      left(cmd_vel_ANALOG_LEFT,cmd_vel_ANALOG_LEFT,cmd_vel_ANALOG_LEFT,cmd_vel_ANALOG_LEFT);
    }
    if(joy.axes[LEFT_ANALOG_RIGHT_LEFT] < 0)
    {
      cmd_vel_ANALOG_LEFT = joy.axes[LEFT_ANALOG_RIGHT_LEFT]*-100;
      cmd_vel_ANALOG_LEFT = map(cmd_vel_ANALOG_LEFT, 0, 99, 0, 180);
      right(cmd_vel_ANALOG_LEFT,cmd_vel_ANALOG_LEFT,cmd_vel_ANALOG_LEFT,cmd_vel_ANALOG_LEFT);
    }
    
    if(joy.axes[RIGHT_ANALOG_UP_DOWN] == 1)
    {
      go(vel,vel,vel,vel);    
    }
    if(joy.axes[RIGHT_ANALOG_UP_DOWN] == -1)
    {
      back(vel,vel,vel,vel);
    }
    if(joy.axes[RIGHT_ANALOG_RIGHT_LEFT] == 1)
    {
      left(vel,vel,vel,vel);
    }
    if(joy.axes[RIGHT_ANALOG_RIGHT_LEFT] == -1)
    {
      right(vel,vel,vel,vel);
    }
    
    if(joy.axes[DPAD_UP_DOWN] == 1)
    {    
      if ((vel + 5)>=30 and (vel + 5)<255)
      {
        vel = vel+5;
        sprintf(vel_on,"%ld", vel);
        nh.loginfo("pwm: ");
        nh.loginfo(vel_on);
      }
    }
    if(joy.axes[DPAD_UP_DOWN] == -1)
    {
      if ((vel - 5)>30 and (vel - 5)<=255)
      {
        vel = vel-5;
        sprintf(vel_on,"%ld", vel);
        nh.loginfo("pwm: ");
        nh.loginfo(vel_on);
      }
    }
    if(joy.axes[DPAD_RIGHT_LEFT] == -1)
    {
      if ((vel + 5)>=30 and (vel + 5)<255)
      {
        vel = vel+5;
        sprintf(vel_on,"%ld", vel);
        nh.loginfo("pwm: ");
        nh.loginfo(vel_on);
      }
    }
    if(joy.axes[DPAD_RIGHT_LEFT] == 1)
    {
      if ((vel - 5)>30 and (vel - 5)<=255)
      {
        vel = vel-5;
        sprintf(vel_on,"%ld", vel);
        nh.loginfo("pwm: ");
        nh.loginfo(vel_on);
      }
    }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////Inicia os subscribers////////////////////////
ros::Subscriber<std_msgs::Int16> pwm_A("cmd_vel_A", &velCallBack_A);
ros::Subscriber<std_msgs::Int16> pwm_B("cmd_vel_B", &velCallBack_B);
ros::Subscriber<std_msgs::Int16> pwm_C("cmd_vel_C", &velCallBack_C);
ros::Subscriber<std_msgs::Int16> pwm_D("cmd_vel_D", &velCallBack_D);
ros::Subscriber<sensor_msgs::Joy> joy("joy", &joyCallBack);

//Inicializa Pinos
void setup(){
//Serial.begin(9600);
pinMode(IN1,OUTPUT);
pinMode(IN2,OUTPUT);
pinMode(IN3,OUTPUT);
pinMode(IN4,OUTPUT);
pinMode(velocidadeA,OUTPUT);
pinMode(velocidadeB,OUTPUT);
pinMode(IN5,OUTPUT);
pinMode(IN6,OUTPUT);
pinMode(IN7,OUTPUT);
pinMode(IN8,OUTPUT);
pinMode(velocidadeC,OUTPUT);
pinMode(velocidadeD,OUTPUT);

//Motor A
pinMode(C1, INPUT_PULLUP);// Canal 1
pinMode(C2, INPUT_PULLUP);// Canal 2
//Motor B
pinMode(C3, INPUT_PULLUP);// Canal 1
pinMode(C4, INPUT_PULLUP);// Canal 2
//Motor C
pinMode(C5, INPUT_PULLUP);// Canal 1
pinMode(C6, INPUT_PULLUP);// Canal 2
//Motor D
pinMode(C7, INPUT_PULLUP);// Canal 1
pinMode(C8, INPUT_PULLUP);// Canal 2

nh.getHardware()->setBaud(115200); //Baudrate de 115200
nh.initNode();
nh.subscribe(pwm_A);
nh.subscribe(pwm_B);
nh.subscribe(pwm_C);
nh.subscribe(pwm_D);
nh.subscribe(joy);

nh.advertise(ticks); //Todos os encoders

//Interrupções do encoder do motor A
attachInterrupt(digitalPinToInterrupt(C1), ai0, RISING); //Canal 1
attachInterrupt(digitalPinToInterrupt(C2), ai1, RISING); //Canal 2
//Interrupçoes do encoder do motor B
attachInterrupt(digitalPinToInterrupt(C3), ai2, RISING); //Canal 1
attachInterrupt(digitalPinToInterrupt(C4), ai3, RISING); //Canal 2
//Interrupções do encoder do motor C
attachInterrupt(digitalPinToInterrupt(C5), ai4, RISING); //Canal 1
attachInterrupt(digitalPinToInterrupt(C6), ai5, RISING); //Canal 2
//Interrupçoes do encoder do motor D
attachInterrupt(digitalPinToInterrupt(C7), ai6, RISING); //Canal 1
attachInterrupt(digitalPinToInterrupt(C8), ai7, RISING); //Canal 2

}

void loop(){
  
  unsigned long currentMillis = millis();
  
  sprintf(motores,"%ld#%ld#%ld#%ld", counter_A, counter_B, counter_C, counter_D);
  ticks_msg.data = motores;  
 
  if (currentMillis-range_timer >= t_pub) //publish every 50 milliseconds
  {
    range_timer = currentMillis+t_pub;
    ticks.publish( &ticks_msg);
  }

  nh.spinOnce();
  
}
