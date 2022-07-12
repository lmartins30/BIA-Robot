#define EN        8  

//Direction pin
#define X_DIR     5 
#define Y_DIR     6

//Step pin
#define X_STP     2
#define Y_STP     3

//Define os pinos para o trigger e echo - meio
#define pino_trigger_m 11
#define pino_echo_m 12

//Define os pinos para o trigger e echo - direita
#define pino_trigger_d 4
#define pino_echo_d 7

//Define os pinos para o trigger e echo - esquerda
#define pino_trigger_e 9
#define pino_echo_e 10

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

#include <Ultrasonic.h>

int v1=0;// velocidade do motor da direita  v maxima 20 / v minima 200 em modulo
int v2=0;// velocidade do motor da esquerda 
long cmE, cmD, cmC;

//Inicializa o sensor nos pinos definidos acima
Ultrasonic ultrasonicm(pino_trigger_m, pino_echo_m);
Ultrasonic ultrasonicd(pino_trigger_d, pino_echo_d);
Ultrasonic ultrasonice(pino_trigger_e, pino_echo_e);
//DRV8825

void MessageFB(const std_msgs::Int32MultiArray& velo)
{
  v1 = velo.data[0];
  v2 = velo.data[1];
}

ros::NodeHandle  nh;

std_msgs::Int32MultiArray str_msg;
std_msgs::Int32MultiArray msg_fb;
int32_t str_data[5];
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<std_msgs::Int32MultiArray> sub("feedback", &MessageFB);


void setup(){

  pinMode(X_DIR, OUTPUT); 
  pinMode(Y_DIR, OUTPUT);
  pinMode(X_STP, OUTPUT);
  pinMode(Y_STP, OUTPUT);
  pinMode(EN, OUTPUT);

  digitalWrite(EN, LOW);  // HIGH desabilita os motores.
  digitalWrite(X_DIR, HIGH);
  digitalWrite(X_DIR, HIGH);
  digitalWrite(X_STP, LOW);
  digitalWrite(Y_STP, LOW);
  

  cli();

  TCCR3A=0;
  TCCR3B=0;
  TCNT3=0;
  
  OCR3A=400;
  TCCR3B|=(1<<WGM32);
  TCCR3B|=(1<<CS30);
  TIMSK3|=(1<<OCIE3A);
  
  sei();
 
  str_msg.data_length=5;
  str_msg.data = str_data;
  
  nh.initNode();
  
  nh.advertise(chatter);
  nh.subscribe(sub);

}

void loop(){
    
  le_sensores();
  
  float vel1=0, vel2=0;
  
  if (v1 != 0)
    vel1 = 80/v1;
    
  if (v2 != 0)
    vel2 = 80/v2;
  
  str_msg.data[0] = vel1;
  str_msg.data[1] = vel2;
  str_msg.data[2] = cmE;
  str_msg.data[3] = cmC;
  str_msg.data[4] = cmD;  

  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(100);

}

ISR(TIMER3_COMPA_vect)
{ 
 static int high1=0;
 static int d1=0;
 static int high2=0;
 static int d2=0;
 
 d1++;
 d2++;
// motor direita 
 if (v1 && d1 >= abs(v1))
 { 
   digitalWrite(Y_DIR, v1 > 0);
   
   if (high1)
   {
    digitalWrite(Y_STP, LOW);
   }
   else
   {
    digitalWrite(Y_STP, HIGH);
  }
  
  high1 = !high1;
  d1 = 0;
 }

//motor esquerda 
if (v2 && d2 >= abs(v2))
 {
   digitalWrite(X_DIR, v2 > 0);
   
   if (high2)
   {
    digitalWrite(X_STP, LOW);
   }
   else
   {
    digitalWrite(X_STP, HIGH);
  }
  
  high2 = !high2;
  d2 = 0;
 }
//if (v1==0) && (v2==0)
// {
//  digitalWrite(En, HIGH): // enabla a placa se os dois sao 0
// }
}

void le_sensores(){

  cmC = ultrasonicm.Ranging(CM);
  cmD = ultrasonicd.Ranging(CM);
  cmE = ultrasonice.Ranging(CM);
  // dist 0 o pulso nao voltou, nao ha obstaculo
  
  if (cmC ==0)
  {
    cmC = 400;
  }
  
  if (cmD ==0)
  {
    cmD = 400;
  }
  
  if (cmE ==0)
  {
    cmE = 400;
  }
  
}
