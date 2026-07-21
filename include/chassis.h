#include <string>
#include <atomic>
#include "pros/motors.h"
#include "pros/motor_group.hpp"
#include "pros/imu.h"
#include "pros/rotation.hpp"
#include "pros/optical.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.h"

const float PI = 3.14159265;

#ifndef CHASSIS_H
#define CHASSIS_H

struct Pose
{
    double x; // inches
    double y;
    double theta; // radians
};

class Chassis
{
public:
    // non-tracking
    Chassis(pros::MotorGroup &leftSide, pros::MotorGroup &rightSide, pros::IMU inertial, float wheelDiam,
            float wheelCircum, float cartRPM, float maxDegPerSec, float gearRatio, float degPerInch,
            float trackWidth, float maxSpeed)
        : leftSide(leftSide),
          rightSide(rightSide),
          inertial(inertial),
          wheelDiam(wheelDiam),
          wheelCircum(wheelCircum),
          cartRPM(cartRPM),
          maxDegPerSec(maxDegPerSec),
          gearRatio(gearRatio),
          degPerInch(degPerInch),
          trackWidth(trackWidth),
          maxSpeed(maxSpeed) {}
    // tracking
    Chassis(pros::MotorGroup &leftSide, pros::MotorGroup &rightSide, pros::IMU inertial, float wheelDiam,
            float wheelCircum, float cartRPM, float maxDegPerSec, float gearRatio, float degPerInch,
            float trackWidth, float maxSpeed, int horizPort, int vertPort, double vertOffset, double horizOffset, double odomWheelDiam)
        : leftSide(leftSide),
          rightSide(rightSide),
          inertial(inertial),
          wheelDiam(wheelDiam),
          wheelCircum(wheelCircum),
          cartRPM(cartRPM),
          maxDegPerSec(maxDegPerSec),
          gearRatio(gearRatio),
          degPerInch(degPerInch),
          trackWidth(trackWidth),
          maxSpeed(maxSpeed),
          vertOffset(vertOffset),
          horizOffset(horizOffset),
          odomWheelDiam(odomWheelDiam)
    {
        // horizEnc.emplace(horizPort);
        verticalEnc.emplace(vertPort);
    }
    void straight(float dist, float startVel, float maxVel, float endVel, float heading, bool useTrackingWheel = false);
    void straightNoHeading(float dist, float startVel, float maxVel, float endVel, bool useTrackingWheel = false);
    void cTurn(float heading, float startVel, float maxVel, float endVel);
    void lSwing(float heading, float startVel, float maxVel, float endVel);
    void rSwing(float heading, float startVel, float maxVel, float endVel);
    void accelTest();
    void distTest();

    Pose getPose();
    void resetPose(double x, double y, double theta);
    void startOdomLoop();
    void turnToPoint(double x, double y, float startVel, float maxVel, float endVel);
    void moveToPoint(double x, double y, float startVel, float maxVel, float endVel);
    void moveToPose(double x, double y, double theta, float startVel, float maxVel, float endVel, float lookAheadDistance = 6.0);

    void strEarlyExitConstants(float g_earlyExitRangeStr, float g_earlyExitTimeStr);
    void turnEarlyExitConstants(float g_earlyExitRangeTurn, float g_earlyExitTimeTurn);
    void swingEarlyExitConstants(float g_earlyExitRangeSwing, float g_earlyExitTimeSwing);

private:
    pros::MotorGroup &leftSide;
    pros::MotorGroup &rightSide;
    pros::IMU inertial;

    void initMP(float dist, float startVel, float maxVel, float endVel, float maxAccel, float maxDecel);
    float getMPdist(float timeElapsed, float maxAccel, float maxDecel);
    float getMPvel(float timeElapsed, float maxAccel, float maxDecel);
    float getMPaccel(float timeElapsed, float maxAccel, float maxDecel);

    // robot measurements
    float wheelDiam;
    float wheelCircum;
    float cartRPM;
    float maxDegPerSec;
    float gearRatio;
    float degPerInch;
    float trackWidth;
    float maxSpeed;

    // robot attributes
    float g_targetHeading = 0.0;

    // exit conditions
    float g_earlyExitRangeStr = 2.0;
    float g_earlyExitTimeStr = 3.0;

    float g_earlyExitRangeTurn = 2.0;
    float g_earlyExitTimeTurn = 1.0;

    float g_earlyExitRangeSwing = 2.0;
    float g_earlyExitTimeSwing = 1.0;

    // motion profile variables
    float g_startTime;
    float g_accelTime;
    float g_cruiseTime;
    float g_decelTime;
    float g_accelDist;
    float g_cruiseDist;
    float g_decelDist;
    float g_cruiseStartTime;
    float g_decelStartTime;
    float g_totTime;
    float g_startVel;
    float g_maxVel;
    float g_endVel;
    float g_totDist;
    float g_distTraveled;

    // motion profile constants
    const float g_accelToMax = 1.0; // time it takes to accelerate to max speed from 0

    const float g_maxLinSpeedPct = 0.97;                                                                          // max speed dampner to account for friction
    const float g_maxLinSpeed = maxSpeed * g_maxLinSpeedPct;                                                      // calculated top speed of robot
    const float g_linAccelPct = 1.1;                                                                              // max acceleration dampner to account for friction and desired max accelertion
    const float g_linMaxAccel = (g_maxLinSpeed / g_accelToMax) * g_linAccelPct; /*76.0 * degPerInch * accelPct;*/ //(wheelCircum * (cartRPM * gearRatio) / 60.0 / accelToMax) * accelPct; //calculated max accelertion
    const float g_linDecelPct = 1.75;                                                                             // max deceleration dampner to account for friction and desired max decelertion
    const float g_linMaxDecel = (-1.0) * (g_maxLinSpeed / g_accelToMax) * g_linDecelPct;                          // calculated max deceleration

    const float g_maxTurnSpeedPct = 0.8;
    const float g_maxTurnSpeed = (maxSpeed / (PI * trackWidth)) * 360.0 * g_maxTurnSpeedPct; // calculate top turning speed of robot
    const float g_turnMaxAccelPct = 0.9;
    const float g_turnMaxAccel = (g_maxTurnSpeed / g_accelToMax) * g_turnMaxAccelPct;
    const float g_turnMaxDecelPct = 1.80;
    const float g_turnMaxDecel = (-1.0) * (g_maxTurnSpeed / g_accelToMax) * g_turnMaxDecelPct;

    const float g_maxSwingSpeedPct = 0.8;
    const float g_maxSwingSpeed = (maxSpeed / (2.0 * PI * trackWidth)) * 360.0 * g_maxSwingSpeedPct;
    const float g_swingMaxAccelPct = 0.9;
    const float g_swingMaxAccel = (g_maxSwingSpeed / g_accelToMax) * g_swingMaxAccelPct;
    const float g_swingMaxDecelPct = 1.8;
    const float g_swingMaxDecel = (-1.0) * (g_maxSwingSpeed / g_accelToMax) * g_swingMaxDecelPct;

    // Tuners
    const float dT = 10.0; // the difference in time, in milliseconds

    float driveKP = 4.7; // main motion profile tuner
    float driveKI = 0.0;
    float driveKD = 0.14;
    float driveKV = 1.3;
    float driveKA = 0.08;

    const float headingKP = 12.0;
    const float headingKI = 0;
    const float headingKD = 0.15;

    const float turnKP = 2.0;
    const float turnKI = 0.0;
    const float turnKD = 0.7;
    const float turnKV = 0.3;
    const float turnKA = 0.0;

    const float swingKP = 2.0;
    const float swingKI = 0;
    const float swingKD = 1.1;
    const float swingKV = 0.4;
    const float swingKA = 0.0;

    const float curveKP = 0.1;
    const float curveKI = 0;
    const float curveKD = 0.1;

    // Odom sensors
    std::optional<pros::Rotation> horizEnc;
    std::optional<pros::Rotation> verticalEnc;

    // Odom constants
    double odomWheelDiam;
    const double ticksPerRev = 36000;
    double vertOffset; // distance from robot center of rotation
    double horizOffset;

    // State
    Pose pose = {0, 0, 0};
    pros::Task *odomTask;
    std::atomic<bool> odomReseedFlag = false;

    // Internal
    static void odomLoop(void *p);
    double ticksToInches(double t);
};

#endif