#define USE_USBCON

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <Encoder.h>


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
long       lastTime;
const double  PI_NUMBER     = 3.14;
const double  WHEEL_DIAMETER  = 0.0695;  //m
const int     CPR       = 1456;

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

  nh.initNode();
  nh.advertise(pubOdometry);
  nh.advertise(pubDebug);
  nh.subscribe(subCmdVelocity);

}

void loop() {

  nh.spinOnce();

  //------------------Publish ODOMETRY--------------------
  struct TwoWheel cmdVelocity = getVelocity();

  if (odoCount > ODO_WAIT) {
    msgOdometry.linear.x = cmdVelocity.left;
    msgOdometry.linear.y = cmdVelocity.right;

    msgDebug.linear.x = currentEffortInput.left;
    msgDebug.linear.y = currentEffortInput.right;

    pubDebug.publish(&msgDebug);
    pubOdometry.publish(&msgOdometry);
  }
  else {
    odoCount++;
  }

  //------------------update wheel speed------------------
  currentEffortInput.left += controlEffort.left;
  currentEffortInput.right += controlEffort.right;

  //------------------APPLY LIMITS------------------------
  //--------------FOR THE LEFT MOTOR----------------------
  if (currentEffortInput.left < 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
    directionLeft = -1;
  }
  else if (currentEffortInput.left >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
    directionLeft = 1;
  }
  
  if (currentEffortInput.left > 255) {
    currentEffortInput.left = 255;
  }
  else if (currentEffortInput.left < -255) {
    currentEffortInput.left = -255;
  }

  //--------------FOR THE RIGHT MOTOR----------------------
  if (currentEffortInput.right < 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
    directionRight = -1;
  }
  else if (currentEffortInput.right >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
    directionRight = 1;
  }
  
  if (currentEffortInput.right > 255) {
    currentEffortInput.right = 255;
  }
  else if (currentEffortInput.right < -255) {
    currentEffortInput.right = -255;
  }

  analogWrite(MOTOR_LEFT_PWM, int(directionLeft * currentEffortInput.left));
  analogWrite(MOTOR_RIGHT_PWM, int(directionRight * currentEffortInput.right));

  delay(100);
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

  //Vitesse tangentielle
  double leftDistance = leftEncoderValue * 3.14 * WHEEL_DIAMETER / CPR;
  velocity.left =  (leftDistance / dtime) * 1000;  //en m/s
  
  double rightDistance = rightEncoderValue * PI_NUMBER * WHEEL_DIAMETER / CPR;
  velocity.right =  (rightDistance / dtime) * 1000;  //en m/s
  
  return velocity;
}
