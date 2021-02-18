#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <string>
 

char motores[80];
long int counter_A = 0, counter_B = 0, counter_C = 0, counter_D = 0;
float up = 0, down = 0, left = 0, right = 0;
bool g = false, b = false, l = false, r = false, p = true;
//Botões do joystick
int A = 0, B = 1, X = 3, Y = 4;//No controle de XBOX A = 0, B = 1, X = 2, Y = 3;
int LB = 6, RB = 7, BACK = 10, START = 11;//No controle de XBOX LB = 4, RB = 5, BACK = 6, START = 7, XBOX = 8;
//Eixos do joystick
int LEFT_ANALOG_UP_DOWN = 1, LEFT_ANALOG_RIGHT_LEFT = 0, RIGHT_ANALOG_UP_DOWN = 3, RIGHT_ANALOG_RIGHT_LEFT = 2, DPAD_UP_DOWN = 7, DPAD_RIGHT_LEFT = 6, RT = 4, LT = 5;


void go(){

  counter_A = counter_A + (up*10);
  counter_B = counter_B + (up*10);
  counter_C = counter_C + (up*10);
  counter_D = counter_D + (up*10);

 }

void back(){

  counter_A = counter_A + (down*10);
  counter_B = counter_B + (down*10);
  counter_C = counter_C + (down*10);
  counter_D = counter_D + (down*10);

}

void left_f(){
  
  counter_A = counter_A + (left*10);
  counter_B = counter_B - (left*10);
  counter_C = counter_C - (left*10);
  counter_D = counter_D + (left*10);
  
}

void right_f(){

  counter_A = counter_A + (right*10);
  counter_B = counter_B - (right*10);
  counter_C = counter_C - (right*10);
  counter_D = counter_D + (right*10);

}


void joy_cb(const sensor_msgs::Joy::ConstPtr& joy){

  if (joy->buttons[X] == 1){
    counter_A = 0, counter_B = 0, counter_C = 0, counter_D = 0;
  }
  if(joy->axes[LEFT_ANALOG_UP_DOWN] > 0)
  {
    up = joy->axes[LEFT_ANALOG_UP_DOWN];
  }
  if(joy->axes[LEFT_ANALOG_UP_DOWN] == 0)
  {
    up = 0, down = 0;  
  }
  if(joy->axes[LEFT_ANALOG_UP_DOWN] < 0)
  {
    down = joy->axes[LEFT_ANALOG_UP_DOWN];
  }
  if(joy->axes[LEFT_ANALOG_RIGHT_LEFT] > 0) 
  {
    left = joy->axes[LEFT_ANALOG_RIGHT_LEFT];
  }
  if(joy->axes[LEFT_ANALOG_RIGHT_LEFT] == 0) 
  {
    left = 0, right = 0; 
  }
  if(joy->axes[LEFT_ANALOG_RIGHT_LEFT] < 0)
  {
    right = joy->axes[LEFT_ANALOG_RIGHT_LEFT];
  }
}
  

int main(int argc, char** argv){
  ros::init(argc, argv, "due_fake_publisher");
  ros::NodeHandle n;
  
  std_msgs::String ticks_msg;

  ros::Publisher ticks_pub = n.advertise<std_msgs::String>("ticks", 50);
  
  ros::Subscriber joys = n.subscribe<sensor_msgs::Joy>("joy", 1000, joy_cb);


  ros::Rate r(20);

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages

    go();
    back();
    left_f();
    right_f();

    sprintf(motores,"%ld#%ld#%ld#%ld", counter_A, counter_B, counter_C, counter_D);
    ticks_msg.data = motores;
    ROS_INFO("Lido: %s", motores);
    ticks_pub.publish(ticks_msg);
 
    r.sleep();
  }
}