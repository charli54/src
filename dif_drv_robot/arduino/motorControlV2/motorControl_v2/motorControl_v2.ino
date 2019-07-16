#define USE_USBCON

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <Encoder.h>

#define ENCODER_RA 2
#define ENCODER_RB 3
#define MOTOR_PWM_R 6
#define MOTOR_DIR_R 7
#define ENCODER_LA 0
#define ENCODER_LB 1
#define MOTOR_PWM_L 5
#define MOTOR_DIR_L 4

#define NOMBRE_PI 3.14
#define WHEEL_DIAMETER 0.06
#define CPR 1456

//float wheelDistance = 0.2;
//float wheelDiameter = 0.06;
//int CPR = 1456;

#define ODO_WAIT 3
int odoCount = 0;


long lastTime = 0;

double vel_x = 0.0;

struct Wheels{
  double left;
  double right;  
};

Wheels velocity_cmd = {0.0, 0.0};
Wheels velocity_odo = {0.0, 0.0};

int current_right_motor_input = 0;

Encoder lEncoder(ENCODER_LA, ENCODER_LB);
Encoder rEncoder(ENCODER_RA, ENCODER_RB);


//ROS Variables **************************
ros::NodeHandle nh;

//Publication de l'odométrie à partir des encoders
geometry_msgs::Twist odoMsg;
ros::Publisher pub_odo("ard_odo", &odoMsg);

//Souscriveur à la vitesse de la roue gauche
void getLeftVelocity(const std_msgs::Float64& l_vel){
	velocity_cmd.left = l_vel.data;
}

ros::Subscriber<std_msgs::Float64> sub_left_velocity("/left_wheel/control_effort", &getLeftVelocity);

//Souscriveur à la vitesse de la roue droite
void getRightVelocity(const std_msgs::Float64& r_vel){
  velocity_cmd.right = r_vel.data;
}

ros::Subscriber<std_msgs::Float64> sub_right_velocity("/right_wheel/control_effort", &getLeftVelocity);

//---------GET VELOCITY FROM ENCODERS------------------
Wheels getVelocityFromEncoders(){

	long leftEncoderValue = lEncoder.read();
	long rightEncoderValue = rEncoder.read();

	lEncoder.write(0);
	rEncoder.write(0);

	long nowTime = millis();
	int deltaTime = nowTime - lastTime;
	lastTime = nowTime;

	double deltaLeftAngle = leftEncoderValue * 1.00 / CPR;
	double deltaRightAngle = rightEncoderValue * 1.00 / CPR;

	double omegaLeft = (deltaLeftAngle / deltaTime) * 2 * NOMBRE_PI * 1000;
	double omegaRight = (deltaRightAngle / deltaTime) * 2 * NOMBRE_PI * 1000;

	double deltaDistanceLeft = (leftEncoderValue * NOMBRE_PI * WHEEL_DIAMETER) / CPR;
	double deltaDistanceRight = (rightEncoderValue * NOMBRE_PI * WHEEL_DIAMETER) / CPR;
	
	Wheels angularVelocity;
	angularVelocity.left = omegaLeft;
	angularVelocity.right = omegaRight;

	return angularVelocity;
}

void setup(){
	pinMode(MOTOR_DIR_L, OUTPUT);
	pinMode(MOTOR_PWM_L, OUTPUT);
	pinMode(MOTOR_DIR_R, OUTPUT);
	pinMode(MOTOR_PWM_L, OUTPUT);

	nh.initNode();
	nh.advertise(pub_odo);
	nh.subscribe(sub_right_velocity);
	nh.subscribe(sub_left_velocity);

	lastTime = millis();
}

void loop(){
	velocity_odo = getVelocityFromEncoders();
	nh.spinOnce();
	if(odoCount > ODO_WAIT){
		odoMsg.linear.x = velocity_odo.left;
		odoMsg.linear.y = velocity_odo.right;

		pub_odo.publish(&odoMsg);
	}
 else{odoCount++;}

	digitalWrite(MOTOR_DIR_R, HIGH);
	analogWrite(MOTOR_PWM_R, 50);

 delay(100);
}
