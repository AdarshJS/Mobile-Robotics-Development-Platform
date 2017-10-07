/*
 *                                                               ## ODOMETRY ##
 * This code updates the location of a two wheeled differential drive robot using two incremental shaft encoders. 
 * 
 * The circumference of the wheels are 30cm and the number of projections(fingers) on the encoder disk are 30 so to reduce computation it is not multiplied and divided when updating
 * distance moved by left and right wheels.If the radius of the wheel or number of projections in the shaft encoder is changed then update distance moved by left and right wheel as 
 * (number of ticks * circumference of wheel)/number of projections on the shaft encoder
 * 
 * Change the value of L if distance between two wheels are altered 
 * 
 * Pins: 4 GPIO pins; 2 PWM pins; 2 analog pins
 * 
 * Libraries required: TimerOne
*/
#include "TimerOne.h"
#include <ros.h>
#include <geometry_msgs/Vector3.h>

geometry_msgs::Vector3 num;

ros::NodeHandle nh;
ros::Publisher p("arduino_odom", &num);


int sensorRight = 0;                          //analog reading from right encoder
int sensorLeft = 0;                           //analog reading from left encoder
int flag1 = 0;                                //to make sure each finger is not counted twice ; set to 1 when finger is not in front of the optical encoder ; left sensor
int flag2 = 0;                                //same as above ; right sensor
int left = 0;                                 //ticks on the left shaft encoder
int right = 0;                                //ticks on the right shaft encoder
double x,y,theta;                              //[x,y,theta] - state of the robot
float x_old,y_old,theta_old;                  //[x_old,y_old,theta_old] - previous state of the robot
double Dr,Dl,Dc;                               //Dr - ditance moved by right wheel; Dl - distance moved by left wheel; Dc - distance moved by the center of the right and left wheels
#define L 27                                   //distance between left and right wheels (#define can be used) this is in cm

float vr = 0.0;
float vl = 0.0;

/*General code for motor and motor driver. Can be reused later.*/
class Motor           
{
  public:
    int pwmPin,input1,input2;                 //pwmPin - pin number to which the enable is connected assuming pwm pulse is given to enable pin; input1 - pin to which A1 is connected; input2 - pin to which A2 is connected 
    int velocity,dir;                         //velocity - PWM value for the motor that is to be set(0-255); dir - direction of motor rotation (1:forward) (-1:backward)
    int sign;                                 //if the wheel moves forward, distance moved by it is positive and if it moves backward the distance moved by it is negative

    Motor(int input1,int input2,int pwmPin)   //constructor indicating pins to which A1,A2 and enable are connected
    {
      this->input1 = input1;
      this->input2 = input2;
      this->pwmPin = pwmPin; 
      pinMode(this->input1,OUTPUT);
      pinMode(this->input2,OUTPUT);
      pinMode(this->pwmPin,OUTPUT);
    }
    
    void set_speed(int velocity)              //to vary the speed of the motor
    {
      this->velocity = velocity;
      analogWrite(this->pwmPin,this->velocity);
    }
    
    void set_direction(int dir)               //to set the direction of the rotation of the motor . [1 = forward] [-1 = backward]
    {
      this->dir = dir;
      if(dir == 1)                            //1 = forward
      {
        digitalWrite(this->input1,HIGH);
        digitalWrite(this->input2,LOW);
        this->sign = 1;
      }
      else if(dir == -1)                      //-1 = backward
      {
        digitalWrite(this->input1,LOW);
        digitalWrite(this->input2,HIGH);
        this->sign=-1;
      }
      else                                    //0 = stop
      {
        digitalWrite(this->input1,LOW);
        digitalWrite(this->input2,LOW);
        this->sign=0;
      }
    }      
};

Motor leftMotor(32,33,9);                       //left motor: A1 to pin 2 in uno; A2 to pin 3; enable pin to pin 9 for pwm
Motor rightMotor(42,43,10);                     //right motor: A1 to pin 4 in uno; A2 to pin 5; enable pin to pin 10

void turn(float turnDir);                       //declared now and defined later

void distance_calc()                          //called every 100ms by timer interrupt
{
/*
 * The equations for odometry update are as follows: 
 *  x_new = x_old + Dc*cos(theta) 
 *  y_new = y_old + Dc*sin(theta)
 *    where Dc = (Dr+Dl)/2
 *  theta_new = theta_old + (Dr-Dl)/L
 *
 * The equations must be updated simultaneously
 * The equations are valid only for small values of theta because only then the assumtion sin(theta) = theta is valid
*/  

  digitalWrite(13,LOW);                       //LED pin is low
  Dr = rightMotor.sign*right;                 //update distance moved by right wheel along with the proper direction (sign); right ticks in right motor
  Dl = leftMotor.sign*left;                   //update distance moved by left wheel along with the proper direction (sign)
  right = left = 0;                           //cleared everytime
  Dc = (Dr+Dl)/2;

  x = x + (Dc*cos(theta));                  //updating the position
  y = y + (Dc*sin(theta));
  theta = theta + (Dr-Dl)/L ;

  //x = x/100;
  //y = y/100;
  //Serial.print(x);
  //Serial.println('\t');
  //Serial.print(y);
  
  num.x = x/100; 
  num.y = y/100;
  num.z = theta;
}

void messageCb(const geometry_msgs::Vector3& msg)  // callback function for arduino_vel
{
  digitalWrite(13,HIGH); // LED glows when data is being subscribed from the laptop
  turn(msg.z); // sets direction to the wheels
  leftMotor.set_speed(int(msg.y));
  rightMotor.set_speed(int(msg.x)); 
     
}

ros::Subscriber<geometry_msgs::Vector3> s("/arduino_vel",messageCb);

void setup()
{ nh.getHardware()->setBaud(115200);
  pinMode(13, OUTPUT);
  // sets direction to the wheels// sets direction to the wheelsleftMotor.set_speed(64);                     //Left motor set to maximum speed i.e. 15 rpm ;
  //rightMotor.set_speed(64);                    //Right motor set to maximum speed i.e. 15 rpm ; 
   
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);
  Serial.begin(115200);
  
  Timer1.initialize(100000);                   
  Timer1.attachInterrupt(distance_calc);
  
}
void loop()
{
  
  sensorLeft=analogRead(A0);                    //The output of left encoder sensor is connected to pin A0
  sensorRight=analogRead(A1);                   //The output of right encoder sensor is connected to pin A1
  
  if(sensorLeft>800&&flag1==1)                  //Experimental threshold is 800 and not counted before
  {
    left++;                                     //left tick incremented  
    flag1=0;                                    //flag indicating that the tick is counted
  }
  else if(sensorLeft<800 && flag1==0)
  {
    flag1=1;                                    //flag indicating that the finger is not detected
  }
  
  if(sensorRight>800 && flag2==1) 
  {
    right++;
    flag2=0;
  }
  else if(sensorRight<800 && flag2==0)
  {
    flag2=1;
  }   
  
  p.publish(&num);
  nh.spinOnce();
  //nh.spinOnce();
  delay(10);  
}

void turn(float turnDir)                          //declared earlier defined now
{
  if(turnDir == 1)                              //1=forward
  {
    leftMotor.set_direction(1);
    rightMotor.set_direction(1);
  }
  else if(turnDir == -1)                        //-1=backward
  {
    leftMotor.set_direction(-1);
    rightMotor.set_direction(-1);
  }
  else if(turnDir == 2)                         //2=right(feel free to modify it)
  {
    leftMotor.set_direction(1);
    rightMotor.set_direction(-1);
  }
  else if(turnDir == -2)                        //-2=left(feel free to modify it)
  {
    leftMotor.set_direction(-1);
    rightMotor.set_direction(1);
  }
}





