// File:          EPuckAvoidCollision.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>

// including sensors lib to use the directional sensors on robot
#include <webots/DistanceSensor.hpp>

// including lib for controlling motor
#include <webots/Motor.hpp>

// defining total time (ms) for a simulation step
// must be multiple of WorldInfo.basicTimeStep
#define TIME_STEP 64

// defining max speed
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();

  // initialize devices

  //  8 distance sensors
  DistanceSensor *ps[8];

  // each string ending with a null terminating char
  // 4 chars of each string
  char psNames[8][4] = {
      "ps0", "ps1", "ps2", "ps3",
      "ps4", "ps5", "ps6", "ps7"};

  // loop to initialize devices
  for (int i = 0; i < 8; i++)
  {
    // link a robot distance sensor to the distance sensor controller
    ps[i] = robot->getDistanceSensor(psNames[i]);

    // enabling the robots distance sensor and refreshing it after every TIME_STEP
    ps[i]->enable(TIME_STEP);
  }

  // initialize motors

  // pointing the motor object controller to robot's motor
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");

  // setting the velocity controlled robot params
  leftMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setPosition(INFINITY);
  rightMotor->setVelocity(0.0);

  // sensor outputs
  double psValues[8];
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1)
  {
    // reading sensor outputs
    for (int i = 0; i < 8; i++)
    {
      psValues[i] = ps[i]->getValue();
    }

    // detect obstacles
    bool right_obstacle = psValues[0] > 80.0 ||
                          psValues[1] > 80.0 || psValues[2] > 80.0;
    bool left_obstacle = psValues[5] > 80.0 ||
                         psValues[6] > 80.0 || psValues[7] > 80.0;

    // initializing motor at 50% of max speed
    double leftSpeed = 0.5 * MAX_SPEED;
    double rightSpeed = 0.5 * MAX_SPEED;

    if (left_obstacle)
    {
      // turn right
      leftSpeed = 0.5 * MAX_SPEED;
      rightSpeed = -0.5 * MAX_SPEED;
    }
    else if (right_obstacle)
    {
      // turn left
      leftSpeed = -0.5 * MAX_SPEED;
      rightSpeed = 0.5 * MAX_SPEED;
    }

    // write actuator inputs
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  }

  delete robot;
  return 0;
}
