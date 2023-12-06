/*---------------------------------------------------------*/
/*                                                         */
/*    Module:       main.cpp                               */
/*    Author:       Robin, AYJ Robotics club               */
/*    Created:      2023 December                          */
/*    Description:  Code for Vex VRC competition bot,      */
/*                  PinWheel DriveTrain - OVER UNDER       */
/*                                                         */
/*---------------------------------------------------------*/

//=============== DEVICE SETUP AND DEFINITION SECTION ==============================================================================================

#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "vex.h"
using namespace vex;
// Brain should be defined by default
brain Brain;

// START V5 MACROS
#define waitUntil(condition) \
    do                       \
    {                        \
        wait(5, msec);       \
    } while (!(condition))

#define repeat(iterations) \
    for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS

// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName)
{
    printf("VEXPlaySound:%s\n", soundName);
    wait(5, msec);
}

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
double turnConstant = 31;
competition Competition = competition();

// This is where you define your devices
controller controller1 = controller(primary);

motor rearLeft = motor(PORT1, ratio18_1, false);
motor frontLeft = motor(PORT2, ratio18_1, false);
motor frontRight = motor(PORT3, ratio18_1, false);
motor rearRight = motor(PORT4, ratio18_1, false);
motor launcher = motor(PORT10, ratio18_1, false);

bumper buttonA = bumper(Brain.ThreeWirePort.A);
bumper buttonB = bumper(Brain.ThreeWirePort.B);
bumper launchSensor = bumper(Brain.ThreeWirePort.H);

//=============== CUSTOM METHOD SECTION =================================================================================================

/**
 * Stops all drivetrain wheel movement
 */
void driveStop()
{
    rearLeft.stop();
    rearRight.stop();
    frontLeft.stop();
    frontRight.stop();
}

/**
 * Drives the robot in the forward direction according to the robot orientation
 * @param time how long the robot should operate for
 * @param speed the speed percentage at which the robot should move at
 */
void driveFoward(double time, double speed)
{
    rearRight.spin(reverse, speed, percent);
    frontLeft.spin(forward, speed, percent);
    frontRight.stop();
    rearLeft.stop();
    wait(time, sec);
    driveStop();
}

/**
 * Drives the robot in the backward direction according to the robot orientation
 * @param time how long the robot should operate for
 * @param speed the speed percentage at which the robot should move at
 */
void driveBackward(double time, double speed)
{
    rearRight.spin(forward, speed, percent);
    frontLeft.spin(reverse, speed, percent);
    frontRight.stop();
    rearLeft.stop();
    wait(time, sec);
    driveStop();
}

/**
 * Drives the robot in the right direction according to the robot orientation
 * @param time how long the robot should operate for
 * @param speed the speed percentage at which the robot should move at
 */
void driveRight(double time, double speed)
{
    rearRight.stop();
    frontLeft.stop();
    frontRight.spin(forward, speed, percent);
    rearLeft.spin(reverse, speed, percent);
    wait(time, sec);
    driveStop();
}

/**
 * Drives the robot in the left direction according to the robot orientation
 * @param time how long the robot should operate for
 * @param speed the speed percentage at which the robot should move at
 */
void driveLeft(double time, double speed)
{
    rearRight.stop();
    frontLeft.stop();
    frontRight.spin(reverse, speed, percent);
    rearLeft.spin(forward, speed, percent);
    wait(time, sec);
    driveStop();
}

/**
 * Turns the robot to the right according to the robot orientation
 * @param time the time it should take for the robot to turn right
 * @param angle the angle at which the robot should turn to in degrees
 */
void turnRight(double time, double angle)
{
    double speed = (turnConstant / 90) * angle / time;
    rearRight.spin(forward, speed, percent);
    frontLeft.spin(forward, speed, percent);
    frontRight.spin(forward, speed, percent);
    rearLeft.spin(forward, speed, percent);
    wait(time, sec);
    driveStop();
}

/**
 * Turns the robot to the left according to the robot orientation
 * @param time the time it should take for the robot to turn left
 * @param angle the angle at which the robot should turn to in degrees
 */
void turnLeft(double time, double angle)
{
    double speed = (turnConstant / 90) * angle / time;
    rearRight.spin(reverse, speed, percent);
    frontLeft.spin(reverse, speed, percent);
    frontRight.spin(reverse, speed, percent);
    rearLeft.spin(reverse, speed, percent);
    wait(time, sec);
    driveStop();
}

/**
 * TWinds the catapult until it releases and triggers a button to stop it
 */
void launch()
{
    while (!launchSensor.pressing())
    {
        launcher.spin(reverse, 40, percent);
    }
    launcher.stop();
}

//=============== AUTONOMOUS METHOD SECTION ==============================================================================================

/**
 * Contains robot commands for what is to be run during autonomous mode
 */
void runOnAutonomous()
{
    // Fill out here
}

//=============== DRIVER CONTROL METHOD SECTION ========================================================================================

/**
 * Contains robot commands for what is to be run during driver control mode
 */
void runOnDriverControl()
{
    Brain.Screen.print("Running driver control");

    // define variable for remote controller enable/disable
    // define variables used for controlling motors based on controller inputs
    bool controller1LeftShoulderControlMotorsStopped = true;
    bool controller1RightShoulderControlMotorsStopped = true;
    bool ExpansionMotorRunning = false;
    // drive bools
    bool isDriveStopped = true;
    bool isLongMovement = true;
    bool isLatMovement = true;
    bool isAngMovement = true;
    // joystick movement vals
    int longitudinalMovement = 0;
    int horizontalMovement = 0;
    int angularMovement = 0;
    // motor movement variables
    int rearLeftMovement = 0;
    int frontLeftMovement = 0;
    int frontRightMovement = 0;
    int rearRightMovement = 0;
    // motor cap
    const int totalSpeed = 80;
    const double speedFactor = 5;

    // loop for robot moving :thumbsup:
    while (true)
    {
        longitudinalMovement = controller1.Axis3.position();
        horizontalMovement = controller1.Axis4.position();
        angularMovement = controller1.Axis1.position();

        // The following code segment deals with the removal of joystick drift: deadbands
        if (longitudinalMovement < 5 && longitudinalMovement > -5)
        {
            if (isLongMovement)
            {
                isLongMovement = false;
                longitudinalMovement = 0;
            }
        }
        else
        {
            isLongMovement = true;
        }

        if (horizontalMovement < 5 && horizontalMovement > -5)
        {
            if (isLatMovement)
            {
                isLatMovement = false;
                horizontalMovement = 0;
            }
        }
        else
        {
            isLatMovement = true;
        }

        if (angularMovement < 5 && angularMovement > -5)
        {
            if (isAngMovement)
            {
                isAngMovement = false;
                angularMovement = 0;
            }
        }
        else
        {
            isAngMovement = true;
        }

        // The following code segment deals with calculating motor speeds based on joystick input
        rearLeftMovement = -horizontalMovement + angularMovement;
        rearLeftMovement /= speedFactor;
        if (rearLeftMovement > totalSpeed)
        {
            rearLeftMovement = totalSpeed;
        }
        else if (rearLeftMovement < -totalSpeed)
        {
            rearLeftMovement = -totalSpeed;
        }

        frontLeftMovement = longitudinalMovement + angularMovement;
        frontLeftMovement /= speedFactor;
        if (frontLeftMovement > totalSpeed)
        {
            frontLeftMovement = totalSpeed;
        }
        else if (frontLeftMovement < -totalSpeed)
        {
            frontLeftMovement = -totalSpeed;
        }

        frontRightMovement = horizontalMovement + angularMovement;
        frontRightMovement /= speedFactor;
        if (frontRightMovement > totalSpeed)
        {
            frontRightMovement = totalSpeed;
        }
        else if (frontRightMovement < -totalSpeed)
        {
            frontRightMovement = -totalSpeed;
        }

        rearRightMovement = -(longitudinalMovement) + angularMovement;
        rearRightMovement /= speedFactor;
        if (rearRightMovement > totalSpeed)
        {
            rearRightMovement = totalSpeed;
        }
        else if (rearRightMovement < -totalSpeed)
        {
            rearRightMovement = -totalSpeed;
        }

        // Optional drivetrain speed cutoff button
        if (controller1.ButtonB.pressing())
        {
            rearLeftMovement /= 4;
            frontLeftMovement /= 4;
            frontRightMovement /= 4;
            rearRightMovement /= 4;
        }

        // The following code segment deals with moving the drive motors using the calculated values
        if (isLongMovement || isLatMovement || isAngMovement)
        {
            rearLeft.setVelocity(rearLeftMovement, percent);
            rearLeft.spin(forward);
            frontLeft.setVelocity(frontLeftMovement, percent);
            frontLeft.spin(forward);
            frontRight.setVelocity(frontRightMovement, percent);
            frontRight.spin(forward);
            rearRight.setVelocity(rearRightMovement, percent);
            rearRight.spin(forward);

            isDriveStopped = false;
        }
        else if (!isDriveStopped)
        { // no drive, stop motors
            rearLeft.stop();
            frontLeft.stop();
            frontRight.stop();
            rearRight.stop();

            isDriveStopped = true;
        }

        // Button A to launch any currently stored objects
        if (controller1.ButtonA.pressing())
        {
            launch();
        }
    }

    // Waits before repeating the process
    wait(20, msec);
}

//=============== MAIN METHOD SECTION ===========================================================================================================

/**
 * Contains all commands that will be executed when robot is activated
 */
int main()
{
    Brain.Screen.print("Robot started.");
    Brain.Screen.newLine();
    // runOnAutonomous();
    runOnDriverControl();
}
