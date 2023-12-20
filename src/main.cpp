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
const double TURN_CONSTANT = 36.5;
const double K = 0.3;
competition Competition = competition();

// This is where you define your devices
controller controller1 = controller(primary);

motor rearLeft = motor(PORT1, ratio18_1, false);
motor frontLeft = motor(PORT2, ratio18_1, false);
motor frontRight = motor(PORT3, ratio18_1, false);
motor rearRight = motor(PORT4, ratio18_1, false);
motor launcher = motor(PORT10, ratio18_1, false);
motor intake = motor(PORT19, ratio36_1, false);
motor test = motor(PORT20, ratio18_1, false);

bumper buttonA = bumper(Brain.ThreeWirePort.A);
bumper buttonB = bumper(Brain.ThreeWirePort.B);
bumper launchSensor = bumper(Brain.ThreeWirePort.H);

vision::signature REDBALL(1, 6467, 8623, 7545, -1037, -345, -691, 3.000, 0);
vision::signature BLUETRIBALL(1, -4929, -3543, -4236, 6779, 10759, 8769, 2.500, 0);
vision::signature SIG_3(3, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_4(4, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_5(5, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_6(6, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_7(7, 0, 0, 0, 0, 0, 0, 3.000, 0);
vex::vision camera = vision(vex::PORT15, 50, REDBALL, BLUETRIBALL, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);

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
    double speed = (TURN_CONSTANT / 90) * angle / time;
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
    double speed = (TURN_CONSTANT / 90) * angle / time;
    rearRight.spin(reverse, speed, percent);
    frontLeft.spin(reverse, speed, percent);
    frontRight.spin(reverse, speed, percent);
    rearLeft.spin(reverse, speed, percent);
    wait(time, sec);
    driveStop();
}

/**
 * Turns the robot dependent on the sign of the value passed in
 * @param speed the speed percentage at which the robot should turn. A positive speed value means turning right.
 */
void freeTurn(double speed)
{
    if (abs(speed) > 20 * K) // Turn robot PROPORTIONALLY
    {
        rearRight.spin(forward, speed, percent);
        frontLeft.spin(forward, speed, percent);
        frontRight.spin(forward, speed, percent);
        rearLeft.spin(forward, speed, percent);
    }
    else
    {
        driveStop();
    }
}

/**
 * Winds the catapult until it releases and triggers a button to stop it
 */
void launch()
{
    while (!launchSensor.pressing())
    {
        launcher.spin(reverse, 40, percent);
    }
    launcher.stop();
}

/**
 * @param speed the percent speed at which the intake should spin inwards at
 */
void in(double speed)
{
    intake.spin(forward, speed, percent);
}

/**
 * @param speed the percent speed at which the intake should pin outwards at
 */
void out(double speed)
{
    intake.spin(reverse, speed, percent);
}

double locateBall(vex::vision::signature s)
{
    // bool printOnce = false;
    double difference = 0;
    camera.takeSnapshot(s);
    if (camera.objectCount > 0) // Is object within view?
    {
        difference = camera.largestObject.centerX - 158; // Finding difference
    }
    return difference;
}

//=============== AUTONOMOUS METHOD SECTION ==============================================================================================

/**
 * Contains robot commands for what is to be run during autonomous mode
 */
void runOnAutonomous()
{

    int difference = 0;

    while (true)
    {
        difference = locateBall(REDBALL);
        printf("%f\n", difference);
        freeTurn(difference * K);
        wait(20, msec);
    }
}

//=============== DRIVER CONTROL METHOD SECTION ========================================================================================

/**
 * Contains robot commands for what is to be run during driver control mode
 */
void runOnDriverControl()
{
    Brain.Screen.print("Running driver control");

    // camera detection
    int difference = 0;
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
    const double speedFactor = 1.25;

    // loop for robot moving :thumbsup:
    while (true)
    {
        longitudinalMovement = controller1.Axis3.position();
        horizontalMovement = controller1.Axis4.position();
        angularMovement = controller1.Axis1.position();

        // if (controller1.ButtonRight.pressing())
        // {
        //     difference = locateBall(REDBALL);
        // }
        // else
        // {
        difference = locateBall(BLUETRIBALL);
        // }

        // Turn on auto detection for redball
        if (controller1.ButtonDown.pressing())
        {
            angularMovement += difference * K;
        }

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
            rearLeftMovement /= 2;
            frontLeftMovement /= 2;
            frontRightMovement /= 2;
            rearRightMovement /= 2;
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

        // Left and Right Bumper for intake
        if (controller1.ButtonL1.pressing())
        {
            in(40);
        }
        else if (controller1.ButtonR1.pressing())
        {
            out(40);
        }
        else
        {
            intake.stop();
        }

        // For testing purposes only
        if (buttonA.pressing())
        {
            test.spin(forward, 30, percent);
        }
        else if (buttonB.pressing())
        {
            test.spin(reverse, 30, percent);
        }

        // wait(20, msec);
    }

    // Waits before repeating the process
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
