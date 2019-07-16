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

void setup(){
	pinMode(MOTOR_DIR_L, OUTPUT);
	pinMode(MOTOR_PWM_L, OUTPUT);
	pinMode(MOTOR_DIR_R, OUTPUT);
	pinMode(MOTOR_PWM_L, OUTPUT);
}

void loop(){
	analogWrite(MOTOR_PWM_L, 200);
}