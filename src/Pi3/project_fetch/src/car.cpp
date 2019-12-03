#include "ros/ros.h"
//#include "ros/console.h"
#include "pigpio.h"
#include <src/libs/Stepper.h>
#include <iostream>
//#include "libs/Stepper.h"
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

//const int stepsPerRevolution = 2048;
//Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);
//Stepper myStepper;

const int analogHigh = 125;
const int analogLow = 0;

const int digitalHigh = 1;
const int digitalLow = 0;

const int speedRight = 2;
const int speedLeft  = 3;
const int rightFront = 15;
const int rightBack  = 18;
const int leftFront  = 14;
const int leftBack   = 4;

const int stepperPin0 = 21;
const int stepperPin1 = 20;
const int stepperPin2 = 16;
const int stepperPin3 = 12;

const int stepsPerRevolution = 2048;
//Stepper stepper = Stepper(stepsPerRevolution, stepperPin0, stepperPin2, stepperPin1, stepperPin3);
Stepper* stepper;
bool armOpen = false;

const int triggerPin = 19;
const int echoPin = 26;
const int samples = 5;

void Forward(int t){
  gpioWrite(speedRight, digitalHigh);
  gpioWrite(speedLeft, digitalHigh);
  gpioWrite( rightBack, 0); // Høyre Bakover
  gpioWrite( rightFront, digitalHigh);  // Høyre Framover
  gpioWrite( leftFront, digitalHigh);  //Venstre Framover
  gpioWrite( leftBack, 0);  // Venstre Bakover

  //delay(t);
  ros::Duration(t / 1000.0).sleep();

  gpioWrite(rightFront, 0);
  gpioWrite(leftFront, 0);

  gpioWrite(speedRight, 0);
  gpioWrite(speedLeft, 0);
}

void Backward(int t){
  gpioWrite(speedRight, digitalHigh);
  gpioWrite(speedLeft, digitalHigh);

  gpioWrite( rightBack, digitalHigh); // Høyre Bakover
  gpioWrite( rightFront, 0);  // Høyre Framover
  gpioWrite( leftFront, 0);  //Venstre Framover
  gpioWrite( leftBack, digitalHigh);  // Venstre Bakover

  //delay(t);
  ros::Duration(t / 1000.0).sleep();

  gpioWrite(rightBack, 0);
  gpioWrite(leftBack, 0);

  gpioWrite(speedRight, 0);
  gpioWrite(speedLeft, 0);
}

void TurnRight(int t){
  gpioPWM(speedLeft, analogHigh);
  gpioPWM(speedRight, analogHigh);

  gpioWrite( rightBack, digitalHigh); // Høyre Bakover
  gpioWrite( rightFront, 0);  // Høyre Framover
  gpioWrite( leftFront, digitalHigh);  //Venstre Framover
  gpioWrite( leftBack, 0);  // Venstre Bakover

  //delay(4.44 * deg);
  //ros::Duration((4.44 * deg) / 1000.0).sleep();
  ros::Duration(t / 1000.0).sleep();

  gpioWrite(rightBack, 0);
  gpioWrite(leftFront, 0);

  gpioWrite(speedLeft, 0);
  gpioWrite(speedRight, 0);
}

void TurnLeft(int t){
  gpioPWM(speedLeft, analogHigh);
  gpioPWM(speedRight, analogHigh);

  gpioWrite( rightBack, 0); // Hï¿½yre Bakover
  gpioWrite( rightFront, digitalHigh);  // Hï¿½yre Framover
  gpioWrite( leftFront, 0);  //Venstre Framover
  gpioWrite( leftBack, digitalHigh);  // Venstre Bakover

  //delay(4.44 * deg);
  //ros::Duration((4.44 * deg) / 1000.0).sleep();
  ros::Duration(t / 1000.0).sleep();

  gpioWrite(rightBack, 0);
  gpioWrite(leftFront, 0);

  gpioWrite(speedLeft, 0);
  gpioWrite(speedRight, 0);
}

void takeInput(const geometry_msgs::Twist::ConstPtr& msg) {
  geometry_msgs::Vector3 linear = msg->linear;
  geometry_msgs::Vector3 angular = msg->angular;

  std::cout << angular.z << std::endl;

  if (linear.x > 0)
    Forward(50);
  else if (linear.x < 0)
    Backward(50);

  else if (angular.z < 0)
    TurnRight(50);
  else if (angular.z > 0)
    TurnLeft(50);
}

unsigned long ultrasonic(int triggerPin, int echoPin, unsigned long stuckThreshold = 500000) {
  gpioWrite(triggerPin, 1);
  gpioDelay(10);
  gpioWrite(triggerPin, 0);

  unsigned long stuckChecker = gpioTick();
  unsigned long pulseStart = 0;
  while(gpioRead(echoPin)==0) {
    pulseStart = gpioTick();
    if (pulseStart - stuckChecker > stuckThreshold)
      return 0;
  }

  stuckChecker = gpioTick();
  unsigned long pulseEnd = 0;
  while(gpioRead(echoPin)==1) {
    pulseEnd = gpioTick();
    if (pulseEnd - stuckChecker > stuckThreshold)
      return 0;
  }

  return pulseEnd - pulseStart;
}

void setArmState(bool open) {
  if (open == armOpen)
    return;
  else {
    //stepper.step((stepsPerRevolution / 2) * (open ? 1 : -1));
    stepper->step((stepsPerRevolution / 2) * (open ? 1 : -1));
    armOpen = open;
  }
}

bool readyClaw(std_srvs::Empty::Request &req,
               std_srvs::Empty::Response &res) {
  setArmState(true);
}

int main(int argc, char **argv) {
  //Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);
  ros::init(argc, argv, "car");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  //std::cout << "lol" << std::endl;
  ROS_INFO("lol");

  gpioInitialise();
  gpioSetMode(speedLeft, PI_OUTPUT);
  gpioSetMode(speedRight, PI_OUTPUT);
  gpioSetMode(rightFront, PI_OUTPUT);
  gpioSetMode(rightBack, PI_OUTPUT);
  gpioSetMode(leftFront, PI_OUTPUT);
  gpioSetMode(leftBack, PI_OUTPUT);

  stepper = new Stepper(stepsPerRevolution, stepperPin0, stepperPin2, stepperPin1, stepperPin3);
  stepper->setSpeed(10);

  gpioSetMode(triggerPin, PI_OUTPUT);
  gpioSetMode(echoPin, PI_INPUT);

  ros::Subscriber sub = n.subscribe("car_ctrl", 2, takeInput);
  ros::ServiceServer srv1 = n.advertiseService("ready_claw", readyClaw);

  unsigned long ultrasonicValues[samples];
  for (int i = 0; i < samples; ++i)
    ultrasonicValues[i] = 0;

  int iterator = 0;
  while(ros::ok()) {
    if (armOpen) {
      unsigned long distance = ultrasonic(triggerPin, echoPin);
      //ultraSonic returns 0 on error, if that happens ignore result
      if (distance != 0) {
         ultrasonicValues[iterator] = distance;
         bool allValuesCorrect = true;
         for(int i = 0; i < samples; i++) {
           if (ultrasonicValues[i] == 0 || ultrasonicValues[i] > 500) {
             allValuesCorrect = false;
	     break;
           }
         }
         if (allValuesCorrect) {
	   setArmState(false);
         }
      }
    }

    ++iterator;
    if (iterator >= samples)
      iterator = 0;
    ros::spinOnce();
    loop_rate.sleep();
  }
}

