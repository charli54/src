#define USE_USBCON

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <Encoder.h>
#include <PID_v1.h>



//----------X IS LEFT
//----------Y IS RIGHT

//--------------ENCODER PINS DEFINITIONS--------------------
#define ENCODER_LEFT_A      0
#define ENCODER_LEFT_B      1
#define ENCODER_RIGHT_A     2
#define ENCODER_RIGHT_B     3

Encoder leftEncoder(ENCODER_LEFT_A, ENCODER_LEFT_B);
Encoder rightEncoder(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

//---------GLOBAL VARIABLE for speed calculation----------
long          lastTime;
long          timeNow;
long          interval = 0;
const double  PI_NUMBER     = 3.14;
const double  WHEEL_DIAMETER  = 0.07;  //m
//const int     CPR       = 1456;
const double     CPR       = 1632.0;

//--------------MOTOR PINS DEFINITIONS--------------------
#define MOTOR_LEFT_DIR      4
#define MOTOR_LEFT_PWM      5
#define MOTOR_RIGHT_PWM     6
#define MOTOR_RIGHT_DIR     7

int directionLeft = 0;
int directionRight = 0;

struct TwoWheel {
  double right;
  double left;
};

struct TwoWheel controlEffort = {0, 0};
struct TwoWheel currentEffortInput = {0, 0};

//----------------ROS GLOBAL VARIABLES------------------
const   int   ODO_WAIT  = 3;
int       odoCount  = 0;

ros::NodeHandle nh;

//Publication de l'odométrie des roues
geometry_msgs::Twist msgOdometry;
ros::Publisher pubOdometry("ard_odo", &msgOdometry);

//------------------------DEBUG---------------------------
geometry_msgs::Twist msgDebug;
ros::Publisher pubDebug("ard_debug", &msgDebug);

//Subscriber to CMD_velocity after PID Control
void setVelocity(const geometry_msgs::Twist & CVel) {
  controlEffort.left = CVel.linear.x;
  controlEffort.right = CVel.linear.y;
}

ros::Subscriber<geometry_msgs::Twist> subCmdVelocity("ard_cmd_vel", &setVelocity );


//------------------------PID------------------------------
//Specify the links and initial tuning parameters
double Kp=100.0, Ki=0.01, Kd=0.1;

//Define Variables we'll be connecting to
double Setpoint_l = 0.0;
double Output_l = 0.0;
double Input_l = 0.0;
PID myPID_left(&Input_l, &Output_l, &Setpoint_l, Kp, Ki, Kd, DIRECT);

double Setpoint_r = 0.0;
double Output_r = 0.0;
double Input_r = 0.0;
PID myPID_right(&Input_r, &Output_r, &Setpoint_r, Kp, Ki, Kd, DIRECT);


void setup() {

  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);

  pinMode(ENCODER_RIGHT_A, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);
  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);

  lastTime = millis();

  controlEffort.left = 0.0;
  controlEffort.right = 0.0;
  
  //turn the PID on
  myPID_left.SetMode(AUTOMATIC);
  myPID_right.SetMode(AUTOMATIC);

  //Set PID limits
  myPID_left.SetOutputLimits(-255.0, 255.0);
  myPID_right.SetOutputLimits(-255.0, 255.0);

  myPID_right.SetSampleTime(10);
  myPID_left.SetSampleTime(10);
  
  nh.initNode();
  nh.advertise(pubOdometry);
  nh.advertise(pubDebug);
  nh.subscribe(subCmdVelocity);

}

void loop() {

nh.spinOnce();
timeNow = millis();
interval = timeNow - lastTime;
if(interval >= 10){

  //------------------Publish ODOMETRY--------------------
  struct TwoWheel cmdVelocity = getVelocity();

  if (odoCount > ODO_WAIT) {
    msgOdometry.linear.x = cmdVelocity.left;
    msgOdometry.linear.y = cmdVelocity.right;

    msgDebug.linear.x = cmdVelocity.left;
    msgDebug.linear.y = cmdVelocity.right;
    msgDebug.angular.x = controlEffort.left;
    msgDebug.angular.y = controlEffort.right;
    
    pubDebug.publish(&msgDebug);
    pubOdometry.publish(&msgOdometry);
  }
  else {
    odoCount++;
  }

  Setpoint_l = controlEffort.left;
  Setpoint_r = controlEffort.right;

  Input_l = cmdVelocity.left;
  Input_r = cmdVelocity.right;

  myPID_left.Compute();
  myPID_right.Compute();

  lastTime = millis();
  
}
  //------------------update wheel speed------------------
  //currentEffortInput.left += controlEffort.left;
  //currentEffortInput.right += controlEffort.right;

  //------------------APPLY LIMITS------------------------
  //--------------FOR THE LEFT MOTOR----------------------
  if (Output_l < 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
    directionLeft = -1;
  }
  else if (Output_l >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
    directionLeft = 1;
  }
  

  //--------------FOR THE RIGHT MOTOR----------------------
  if (Output_r < 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
    directionRight = -1;
  }
  else if (Output_r >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
    directionRight = 1;
  }
  
  
  analogWrite(MOTOR_LEFT_PWM, int(directionLeft * Output_l));
  analogWrite(MOTOR_RIGHT_PWM, int(directionRight * Output_r));

  //delay(100);
}

TwoWheel getVelocity() {
  struct TwoWheel velocity = {0, 0};

  long leftEncoderValue = leftEncoder.read();
  long rightEncoderValue = rightEncoder.read();

  leftEncoder.write(0);
  rightEncoder.write(0);

  long nowTime = millis();

  long dtime = nowTime - lastTime;
  lastTime = nowTime;

  //double dAngle = leftEncoderValue * 1.00 / CPR;   //en RPM
  //double omega = (dAngle / dtime) * 1000 * 60;

  /*Vitesse tangentielle
  double leftDistance = leftEncoderValue * 3.14 * WHEEL_DIAMETER / CPR;
  velocity.left =  (leftDistance / dtime) * 1000;  //en m/s
  
  double rightDistance = rightEncoderValue * PI_NUMBER * WHEEL_DIAMETER / CPR;
  velocity.right =  (rightDistance / dtime) * 1000;  //en m/s
  */

  velocity.left  = double(leftEncoderValue * 200.0 * PI_NUMBER) /  (interval * CPR);
  velocity.right = double(rightEncoderValue * 200.0 * PI_NUMBER) / (interval * CPR);
  
  return velocity;
}
