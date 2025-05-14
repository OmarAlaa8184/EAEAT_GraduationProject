
#include <ros.h>
#include <geometry_msgs/Twist.h> //For cmd_vel subscription
#include <geometry_msgs/Point32.h> //For cmd_vel subscription
#include <CytronMotorDriver.h>
#include <Encoder.h>





const int MOTOR_1 = 0;
const int MOTOR_2 = 1;
const int CW = 1;
const int CCW = 2;
const int S = 0;

// Configure the motor driver.
CytronMD motor1(PWM_DIR, 5, 41);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 8, 40); // PWM 2 = Pin 9, DIR 2 = Pin 10.
int motor_State;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder lEnc(3, 2);
Encoder rEnc(18, 19);
//   avoid using pins with LEDs attached

//-----------------------------------------Robot parameters definition------------
#define L 0.46
#define R 0.035

//----------------------------------encoder------------------------------
long roldPosition  = -999;
long loldPosition  = -999;

//--------------------------------Motors VARS-----------------------------------
// initializing variables
float vel = 0.0; //as Twist msgs depend on Vector3 which is float64
float omega = 0.0;
float VR, VL;


//-----------------------------------------------------------------------------------------
ros::NodeHandle  nh;
//------------------------------Publish init----------------------------------------------
geometry_msgs::Point32 Point_msg;
ros::Publisher enc_pub("/encoder", &Point_msg);


//-----------------------------------DC Motors Callback subscribers
void motors_cb(const geometry_msgs::Twist& msg)
{
  vel = msg.linear.x;
  omega = msg.angular.z;

  VR=(2*vel+omega*L)/(2*R)*200; //  convert vel to pulses
  VL=(2*vel-omega*L)/(2*R)*200; 
    
  //-----right motor------
  if (VR < 0)
  {
    motor_State = CW;
    Motor_Cmd(MOTOR_1, motor_State, abs(VR));
  }
  else if (VR > 0)
  {
    motor_State = CCW;
    Motor_Cmd(MOTOR_1, motor_State, abs(VR));
  }else if (VR == 0){
    motor1.setSpeed(0);
    }
  //-----left motor------
  if (VL < 0)
  {
    motor_State = CW;
    Motor_Cmd(MOTOR_2, motor_State, abs(VL));
  }
  else if (VL > 0)
  {
    motor_State = CCW;
    Motor_Cmd(MOTOR_2, motor_State, abs(VL));
  }else if (VL == 0){
    motor2.setSpeed(0);
    }
 
}


    


//--------------------subscribers---------------------------
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motors_cb);

void setup() {
  Serial.begin(57600);

 
  
//---------------------------ROS Setup
      nh.advertise(enc_pub);  
      nh.subscribe(sub);
  
}


void loop() {
//Right Encoder
  long rnewPosition = rEnc.read();
  if (rnewPosition != roldPosition) {
    roldPosition = rnewPosition;
    Serial.println(rnewPosition);
  }
  //----left encoder
  long lnewPosition = lEnc.read();
  if (lnewPosition != loldPosition) {
    loldPosition = lnewPosition; //update positions
    Serial.println(lnewPosition);
  }

  //-------end of encoder
 
 //-----------------------ROS publishing  
        Point_msg.x=rnewPosition;
        Point_msg.y=lnewPosition;
        enc_pub.publish(&Point_msg);
        
  //-------------        
    nh.spinOnce(); 
    delay(10);

  
}


    
void Motor_Cmd(int motor, int DIR, int PWM)     //Function that writes to the motors
{
  if (motor == MOTOR_1)
  {
    if (DIR == CW)    {
      motor1.setSpeed(abs(PWM));
 
    }
    else if (DIR == CCW)    {
      motor1.setSpeed(-abs(PWM));
    }
    else    {
      motor1.setSpeed(0);
    }
  }
  else if (motor == MOTOR_2)
  {
    if (DIR == CW)    {
      motor2.setSpeed(abs(PWM));

    }
    else if (DIR == CCW)    {
      motor2.setSpeed(-abs(PWM));

    }
    else    {
      motor2.setSpeed(0);

    }  }
}
