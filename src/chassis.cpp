#include "main.h"
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <string>
#include <system_error>
#include <atomic>
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/imu.h"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp" Z
#include "chassis.h"

using std::string;

void Chassis::initMP(float dist, float startVel, float maxVel, float endVel, float maxAccel, float maxDecel)
{
    int moveSign = sign(dist);
    // for future use
    g_accelTime = (maxVel - startVel) / maxAccel;
    // finding acceleration time using equation vf = vi + at
    g_decelTime = (endVel - maxVel) / (maxDecel);
    // same for deceleration time
    g_accelDist = startVel * g_accelTime + 0.5 * maxAccel * pow(g_accelTime, 2);
    // finding acceleration distance using equation x = vf t - 0.5a t2
    g_decelDist = maxVel * g_decelTime + 0.5 * (maxDecel)*pow(g_decelTime, 2);
    // same for deceleration distance
    g_cruiseDist = dist - g_accelDist - g_decelDist;
    // find cruising distance
    g_cruiseTime = g_cruiseDist / maxVel;
    // finding cruising time using equation x = vt

    if (g_cruiseDist * moveSign < 0)
    { // edge case for when there is no time to cruise. Must then make triangular motion profile
        // reduces distance, time, max velocities, and eliminates cruising due to lack of distance
        g_accelDist = dist * (g_accelDist / (g_accelDist + g_decelDist));
        // subtracting half distance from both acceleration and deceleration.
        // This only works because acceleration and deceleration have the same magnitude
        g_decelDist = dist * (g_decelDist / (g_accelDist + g_decelDist)); // same for deceleration distance
        if (maxAccel > 0)
        {
            g_accelTime = ((-startVel) + (sqrt(pow(startVel, 2) + 2 * maxAccel * g_accelDist))) / maxAccel;
            // finding new acceleration time for this edge case, using equation x = 0.5(vi + vf)t
        }
        else
        {
            g_accelTime = ((-startVel) - (sqrt(pow(startVel, 2) + 2 * maxAccel * g_accelDist))) / maxAccel;
        }
        g_maxVel = startVel + maxAccel * g_accelTime;
        g_decelTime = (g_decelDist * 2) / (maxVel + endVel);
        // same for deceleration time
        g_cruiseDist = 0;
        // reset cruising distance so the motion profile is triangular
        g_cruiseTime = 0;
        // same for cruising time
    }

    g_cruiseStartTime = g_accelTime;
    g_decelStartTime = g_cruiseStartTime + g_cruiseTime;
    g_totTime = g_decelStartTime + g_decelTime;
    g_totDist = dist;
}

float Chassis::getMPdist(float timeElapsed, float maxAccel, float maxDecel)
{
    if (timeElapsed > g_totTime)
    { // Motion Profile has ended, so target distance is final distance
        return g_totDist;
    }
    else if (timeElapsed < 0)
    { // before the Motion Profile needs to be entered, will not ever come here
        return 0;
    }
    else if (timeElapsed < g_cruiseStartTime)
    { // during acceleration phase
        return g_startVel * timeElapsed + 0.5 * maxAccel * pow(timeElapsed, 2);
    }
    else if (timeElapsed < g_decelStartTime)
    { // during cruising
        return g_accelDist + g_maxVel * (timeElapsed - g_cruiseStartTime);
    }
    else if (timeElapsed < g_totTime)
    { // during deceleration phase
        return g_accelDist + g_cruiseDist + g_maxVel * (timeElapsed - g_decelStartTime) + 0.5 * (maxDecel)*pow((timeElapsed - g_decelStartTime), 2);
    }
    else
    {                     // should never be here
        return g_totDist; // to end this movement if this case is somehow reached
    }
}

float Chassis::getMPvel(float timeElapsed, float maxAccel, float maxDecel)
{
    if (timeElapsed > g_totTime)
    { // Motion Profile has ended, so target velocity is max velocity
        return g_endVel;
    }
    else if (timeElapsed < 0)
    { // before the Motion Profile needs to be entered, will not ever come here.
        return g_startVel;
    }
    else if (timeElapsed < g_cruiseStartTime)
    { // during acceleration phase
        return g_startVel + maxAccel * timeElapsed;
    }
    else if (timeElapsed < g_decelStartTime)
    { // during cruising
        return g_maxVel;
    }
    else if (timeElapsed < g_totTime)
    { // during deceleration phase
        return g_maxVel + maxDecel * (timeElapsed - g_decelStartTime);
    }
    else
    {                    // should never be here
        return g_maxVel; // to end this movement if this case is somehow reached
    }
}

float Chassis::getMPaccel(float timeElapsed, float maxAccel, float maxDecel)
{
    if (timeElapsed > g_totTime)
    { // Motion Profile has ended, so target acceleration is max acceleration
        return maxDecel;
    }
    else if (timeElapsed < 0)
    { // before the Motion Profile needs to be entered, will not ever come here.
        return maxAccel;
    }
    else if (timeElapsed < g_cruiseStartTime)
    { // during acceleration phase
        return maxAccel;
    }
    else if (timeElapsed < g_decelStartTime)
    { // during cruising
        return 0;
    }
    else if (timeElapsed < g_totTime)
    { // during deceleration phase
        return maxDecel;
    }
    else
    {                    // should never be here
        return maxAccel; // to end this movement if this case is somehow reached
    }
}

void Chassis::straight(float dist, float startVel, float maxVel, float endVel, float heading, bool useTrackingWheel)
{
    // get start time
    g_startTime = pros::millis() / 1000.0;
    // If heading is not provided (NaN), default to g_targetHeading
    if (std::isnan(heading))
    {
        heading = g_targetHeading;
    }
    // get movement direction
    int moveSign = sign(dist);

    // convert values
    g_totDist = dist;
    g_startVel = (startVel / 100.0) * g_maxLinSpeed * moveSign;
    g_maxVel = (maxVel / 100.0) * g_maxLinSpeed * moveSign;
    g_endVel = (endVel / 100.0) * g_maxLinSpeed * moveSign;

    // initialize motion profile
    initMP(g_totDist, g_startVel, g_maxVel, g_endVel, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);

    // initialize motor power controllers
    float targetDist;
    float targetVel;
    float targetAccel;
    float leftPower;
    float rightPower;

    // initial conditions
    float leftStartPos = leftSide.get_position();
    float rightStartPos = rightSide.get_position();
    float trackingStartPos;
    if (useTrackingWheel)
    {
        trackingStartPos = verticalEnc->get_position();
    }
    g_distTraveled = 0;
    float distError = g_totDist;

    // errors
    float leftError = 0;
    float leftTotError = 0;
    float leftPrevError = 0;
    float rightError = 0;
    float rightTotError = 0;
    float rightPrevError = 0;

    float headingError = 0;
    float degDiff = 0;

    while (true)
    {
        headingError = inertial.get_heading() - heading; // heading error
        if (fabs(headingError) > 180.0)
        { // checks for large errors from values being on opposite ends of 360 degrees
            headingError = (360.0 - fabs(headingError)) * sign(headingError * (-1.0));
        }
        degDiff = (PI * trackWidth) * (headingError / 360.0) * headingKP;
        // calculates extra distance needed to travel to make up heading error

        targetDist = getMPdist((pros::millis() / 1000.0) - g_startTime, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);
        // find target distance for motors to travel to
        targetVel = getMPvel((pros::millis() / 1000.0) - g_startTime, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);
        // find target velocity for motors to travel at
        targetAccel = getMPaccel((pros::millis() / 1000.0) - g_startTime, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);
        // find target acceleration for motors to travel at
        if (useTrackingWheel)
        {
            float trackingDist = ticksToInches(verticalEnc->get_position() - trackingStartPos);
            // left side error
            leftError = targetDist - degDiff - trackingDist;
            // right side error
            rightError = targetDist + degDiff - trackingDist;
        }
        else
        {
            // left side error
            leftError = targetDist - degDiff - ((leftSide.get_position() - leftStartPos) / degPerInch);
            // right side error
            rightError = targetDist + degDiff - ((rightSide.get_position() - rightStartPos) / degPerInch);
        }
        // combine error and constants
        float feedforward = targetVel * driveKV + targetAccel * driveKA;

        float leftFeedback = leftError * driveKP + leftTotError * driveKI + ((leftError - leftPrevError) / (dT / 1000.0)) * driveKD;
        leftPower = leftFeedback + feedforward;

        float rightFeedback = rightError * driveKP + rightTotError * driveKI + ((rightError - rightPrevError) / (dT / 1000.0)) * driveKD;
        rightPower = rightFeedback + feedforward;

        leftSide.move(leftPower);
        rightSide.move(rightPower);

        if (useTrackingWheel)
        {
            g_distTraveled = ticksToInches(verticalEnc->get_position() - trackingStartPos);
        }
        else
        {
            g_distTraveled = (((leftSide.get_position() - leftStartPos) + (rightSide.get_position() - rightStartPos)) / 2.0) / degPerInch;
        }
        distError = g_totDist - g_distTraveled;

        leftTotError += leftError;
        leftPrevError = leftError;
        rightTotError += rightError;
        rightPrevError = rightError;

        // distance debug
        // pros::lcd::set_text(5, std::format("{}, {}", distError, g_distTraveled));

        if ((fabs(distError) <= g_earlyExitRangeStr) || (pros::millis() / 1000.0 >= g_startTime + g_earlyExitTimeStr))
        {
            break;
        }

        // delay to avoid busy loop
        pros::delay(dT);
    }
    // brake at end
    leftSide.move(0);
    rightSide.move(0);
}

void Chassis::straightNoHeading(float dist, float startVel, float maxVel, float endVel, bool useTrackingWheel)
{
    // get start time
    g_startTime = pros::millis() / 1000.0;

    // get movement direction
    int moveSign = sign(dist);

    // convert values
    g_totDist = dist /* * degPerInch*/;
    g_startVel = (startVel / 100.0) * g_maxLinSpeed * moveSign;
    g_maxVel = (maxVel / 100.0) * g_maxLinSpeed * moveSign; // max velocity may be changed due to insufficient distance, this would be done in Motion Profile initialization
    g_endVel = (endVel / 100.0) * g_maxLinSpeed * moveSign;

    initMP(g_totDist, g_startVel, g_maxVel, g_endVel, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);

    // initialize motor power controllers
    float targetDist;
    float targetVel;
    float targetAccel;
    float leftPower;
    float rightPower;

    // initial conditions
    float leftStartPos = leftSide.get_position();
    float rightStartPos = rightSide.get_position();
    float trackingStartPos;
    if (useTrackingWheel)
    {
        trackingStartPos = verticalEnc->get_position();
    }
    g_distTraveled = 0;
    float distError = g_totDist;

    // errors
    float leftError = 0;
    float leftTotError = 0;
    float leftPrevError = 0;
    float rightError = 0;
    float rightTotError = 0;
    float rightPrevError = 0;

    while (true)
    {
        targetDist = getMPdist((pros::millis() / 1000.0) - g_startTime, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);
        // find target distance for motors to travel to
        targetVel = getMPvel((pros::millis() / 1000.0) - g_startTime, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);
        // find target velocity for motors to travel at
        targetAccel = getMPaccel((pros::millis() / 1000.0) - g_startTime, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);
        // find target acceleration for motors to travel at
        leftError = targetDist - ((leftSide.get_position() - leftStartPos) / degPerInch);
        rightError = targetDist - ((rightSide.get_position() - rightStartPos) / degPerInch);
        leftPower = leftError * driveKP + leftTotError * driveKI + ((leftError - leftPrevError) / (dT / 1000.0)) * driveKD + targetVel * driveKV + targetAccel * driveKA;
        rightPower = rightError * driveKP + rightTotError * driveKI + ((rightError - rightPrevError) / (dT / 1000.0)) * driveKD + targetVel * driveKV + targetAccel * driveKA;

        leftSide.move(leftPower);
        rightSide.move(rightPower);

        if (useTrackingWheel)
        {
            g_distTraveled = ticksToInches(verticalEnc->get_position() - trackingStartPos);
        }
        else
        {
            g_distTraveled = (((leftSide.get_position() - leftStartPos) + (rightSide.get_position() - rightStartPos)) / 2.0) / degPerInch;
        }
        distError = g_totDist - g_distTraveled;

        leftTotError += leftError;
        leftPrevError = leftError;
        rightTotError += rightError;
        rightPrevError = rightError;

        if ((distError * moveSign <= g_earlyExitRangeStr * moveSign) || (pros::millis() / 1000.0 >= g_startTime + g_earlyExitTimeStr))
        {
            break;
        }

        pros::delay(dT);
    }
    leftSide.move(0);
    rightSide.move(0);
}

void Chassis::cTurn(float heading, float startVel, float maxVel, float endVel)
{
    // get start time
    g_startTime = pros::millis() / 1000.0;

    // calculate total distance to travel
    g_targetHeading = heading;
    g_totDist = g_targetHeading - inertial.get_heading();
    if (fabs(g_totDist) > 180)
    { // checks for large errors from values being on opposite ends of 360 degrees
        g_totDist = (360.0 - fabs(g_totDist)) * sign(g_totDist * (-1.0));
    }

    // get movement direction
    int moveSign = sign(g_totDist);

    // convert values
    g_startVel = (startVel / 100.0) * g_maxTurnSpeed * moveSign;
    g_maxVel = (maxVel / 100.0) * g_maxTurnSpeed * moveSign;
    // max velocity may be changed due to insufficient distance, in which case there would be no cruise period
    g_endVel = (endVel / 100.0) * g_maxTurnSpeed * moveSign;

    // initialize motion profile
    initMP(g_totDist, g_startVel, g_maxVel, g_endVel, g_turnMaxAccel * moveSign, g_turnMaxDecel * moveSign);

    // initial conditions
    float startPos = inertial.get_heading();

    // initialize mp controllers
    float targetDist = 0;
    float targetVel = 0;
    float targetAccel = 0;
    float power = 0;

    // errors
    float curHeadingError = 0;
    float headingError = 0;
    float totError = 0;
    float prevError = 0;

    g_distTraveled = 0;

    while (true)
    {
        targetDist = getMPdist((pros::millis() / 1000.0) - g_startTime, g_turnMaxAccel * moveSign, g_turnMaxDecel * moveSign);
        // find target distance for motors to travel to
        targetVel = getMPvel((pros::millis() / 1000.0) - g_startTime, g_turnMaxAccel * moveSign, g_turnMaxDecel * moveSign);
        // find target velocity for motors to travel at
        targetAccel = getMPaccel((pros::millis() / 1000.0) - g_startTime, g_turnMaxAccel * moveSign, g_turnMaxDecel * moveSign);
        // find target acceleration for motors to travel at

        curHeadingError = targetDist - (inertial.get_heading() - startPos);
        if (fabs(curHeadingError) > 180)
        { // checks for large errors from values being on opposite ends of 360 degrees
            curHeadingError = (360.0 - fabs(curHeadingError)) * sign(curHeadingError * (-1.0));
        }

        // power calculations from mp
        power = curHeadingError * turnKP + totError * turnKI + (curHeadingError - prevError) / (dT / 1000.0) * turnKD + targetVel * turnKV + targetAccel * turnKA;

        leftSide.move(power);
        rightSide.move(-power);

        // error assignment for I and D
        totError += curHeadingError;
        prevError = curHeadingError;

        // exit condition calculations
        headingError = g_targetHeading - inertial.get_heading();
        if (fabs(headingError) > 180)
        {
            headingError = (360.0 - fabs(headingError)) * sign(headingError * (-1.0));
        }
        if ((fabs(headingError) <= g_earlyExitRangeTurn) || (pros::millis() / 1000.0 >= g_startTime + g_earlyExitTimeTurn))
        {
            break;
        }

        // delay time
        pros::delay(dT);
    }
    // leftSide.move(0);
    // rightSide.move(0);
    leftSide.set_brake_mode(pros::MotorBrake::hold);
    rightSide.set_brake_mode(pros::MotorBrake::hold);
    leftSide.brake();
    rightSide.brake();
}

void Chassis::lSwing(float heading, float startVel, float maxVel, float endVel)
{
    // get start time
    g_startTime = pros::millis() / 1000.0;

    // calculate total distance to travel
    g_targetHeading = heading;
    g_totDist = g_targetHeading - inertial.get_heading();
    if (fabs(g_totDist) > 180)
    { // checks for large errors from values being on opposite ends of 360 degrees
        g_totDist = (360.0 - fabs(g_totDist)) * sign(g_totDist * (-1.0));
    }

    // get movement direction
    int moveSign = sign(g_totDist);

    // convert values
    g_startVel = (startVel / 100.0) * g_maxSwingSpeed * moveSign;
    g_maxVel = (maxVel / 100.0) * g_maxSwingSpeed * moveSign; // max velocity may be changed due to insufficient distance, this would be done in Motion Profile initialization
    g_endVel = (endVel / 100.0) * g_maxSwingSpeed * moveSign;

    // initialize motion profile
    initMP(g_totDist, g_startVel, g_maxVel, g_endVel, g_swingMaxAccel * moveSign, g_swingMaxDecel * moveSign);

    // initial conditions
    float startPos = inertial.get_heading();
    rightSide.set_brake_mode(pros::MotorBrake::hold);

    // initialize motor power controllers
    float targetDist = 0;
    float targetVel = 0;
    float targetAccel = 0;
    float power = 0;

    // errors
    float curHeadingError = 0;
    float headingError = 0;
    float totError = 0;
    float prevError = 0;

    g_distTraveled = 0;

    while (true)
    {
        targetDist = getMPdist((pros::millis() / 1000.0) - g_startTime, g_swingMaxAccel * moveSign, g_swingMaxDecel * moveSign);   // find target distance for motors to travel to
        targetVel = getMPvel((pros::millis() / 1000.0) - g_startTime, g_swingMaxAccel * moveSign, g_swingMaxDecel * moveSign);     // find target velocity for motors to travel at
        targetAccel = getMPaccel((pros::millis() / 1000.0) - g_startTime, g_swingMaxAccel * moveSign, g_swingMaxDecel * moveSign); // find target acceleration for motors to travel at

        curHeadingError = targetDist - (inertial.get_heading() - startPos);
        if (fabs(curHeadingError) > 180)
        { // checks for large errors from values being on opposite ends of 360 degrees
            curHeadingError = (360.0 - fabs(curHeadingError)) * sign(curHeadingError * (-1.0));
        }

        // power calculations from mp
        power = curHeadingError * swingKP + totError * swingKI + (curHeadingError - prevError) / (dT / 1000.0) * swingKD + targetVel * swingKV + targetAccel * swingKA;

        leftSide.move(power);
        rightSide.brake();
        // opposite for right swing

        // error assignment for I and D
        totError += curHeadingError;
        prevError = curHeadingError;

        // exit condition calculations
        headingError = g_targetHeading - inertial.get_heading();
        if (fabs(headingError) > 180)
        {
            headingError = (360.0 - fabs(headingError)) * sign(headingError * (-1.0));
        }
        if ((fabs(headingError) <= g_earlyExitRangeSwing) || (pros::millis() / 1000.0 >= g_startTime + g_earlyExitTimeSwing))
        {
            break;
        }

        // delay time
        pros::delay(dT);
    }
    leftSide.move(0);
    rightSide.move(0);
}

void Chassis::rSwing(float heading, float startVel, float maxVel, float endVel)
{
    // get start time
    g_startTime = pros::millis() / 1000.0;

    // calculate total distance to travel
    g_targetHeading = heading;
    g_totDist = g_targetHeading - inertial.get_heading();
    if (fabs(g_totDist) > 180)
    { // checks for large errors from values being on opposite ends of 360 degrees
        g_totDist = (360.0 - fabs(g_totDist)) * sign(g_totDist * (-1.0));
    }

    // get movement direction
    int moveSign = sign(g_totDist);

    // convert values
    g_startVel = (startVel / 100.0) * g_maxSwingSpeed * moveSign;
    g_maxVel = (maxVel / 100.0) * g_maxSwingSpeed * moveSign; // max velocity may be changed due to insufficient distance, this would be done in Motion Profile initialization
    g_endVel = (endVel / 100.0) * g_maxSwingSpeed * moveSign;

    // initialize motion profile
    initMP(g_totDist, g_startVel, g_maxVel, g_endVel, g_swingMaxAccel * moveSign, g_swingMaxDecel * moveSign);

    // initial conditions
    float startPos = inertial.get_heading();
    leftSide.set_brake_mode(pros::MotorBrake::hold);

    // initialize motor power controllers
    float targetDist = 0;
    float targetVel = 0;
    float targetAccel = 0;
    float power = 0;

    // errors
    float curHeadingError = 0;
    float headingError = 0;
    float totError = 0;
    float prevError = 0;

    g_distTraveled = 0;

    while (true)
    {
        targetDist = getMPdist((pros::millis() / 1000.0) - g_startTime, g_swingMaxAccel * moveSign, g_swingMaxDecel * moveSign);   // find target distance for motors to travel to
        targetVel = getMPvel((pros::millis() / 1000.0) - g_startTime, g_swingMaxAccel * moveSign, g_swingMaxDecel * moveSign);     // find target velocity for motors to travel at
        targetAccel = getMPaccel((pros::millis() / 1000.0) - g_startTime, g_swingMaxAccel * moveSign, g_swingMaxDecel * moveSign); // find target acceleration for motors to travel at

        curHeadingError = targetDist - (inertial.get_heading() - startPos);
        if (fabs(curHeadingError) > 180)
        { // checks for large errors from values being on opposite ends of 360 degrees
            curHeadingError = (360.0 - fabs(curHeadingError)) * sign(curHeadingError * (-1.0));
        }

        // power calculations from mp
        power = curHeadingError * swingKP + totError * swingKI + (curHeadingError - prevError) / (dT / 1000.0) * swingKD + targetVel * swingKV + targetAccel * swingKA;

        leftSide.brake();
        rightSide.move(-power);

        // error assignment for I and D
        totError += curHeadingError;
        prevError = curHeadingError;

        // exit condition calculations
        headingError = g_targetHeading - inertial.get_heading();
        if (fabs(headingError) > 180)
        {
            headingError = (360.0 - fabs(headingError)) * sign(headingError * (-1.0));
        }
        if ((fabs(headingError) <= g_earlyExitRangeSwing) || (pros::millis() / 1000.0 >= g_startTime + g_earlyExitTimeSwing))
        {
            break;
        }

        // delay time
        pros::delay(dT);
    }
    leftSide.move(0);
    rightSide.move(0);
}

void Chassis::accelTest()
{
    float startTime = pros::millis();
    rightSide.move(127);
    leftSide.move(127);
    float maxSpeedR = 0;
    float maxSpeedL = 0;
    float maxSpeedTime = 0;
    float maxDist = 0;
    while (true)
    {
        if (rightSide.get_actual_velocity() > maxSpeedR)
        {
            maxSpeedR = rightSide.get_actual_velocity();
            maxSpeedL = leftSide.get_actual_velocity();
            maxSpeedTime = pros::millis() - startTime;
            maxDist = rightSide.get_position() / degPerInch;
        }
        string maxspeed = std::to_string(int(maxSpeedR)) + ", " + std::to_string(int(maxSpeedL));
        string maxspeedTime = std::to_string(int(maxSpeedTime));
        string maxdist = std::to_string(int(maxDist));
        string currDist = std::to_string(int(rightSide.get_position() / degPerInch));

        if (pros::millis() >= 900 + startTime)
        {
            break;
        }
    }

    rightSide.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftSide.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightSide.brake();
    leftSide.brake();
}

void Chassis::distTest()
{
    rightSide.set_zero_position(0);
    leftSide.set_zero_position(0);
    while (true)
    {
        string leftVal = std::to_string(int(leftSide.get_position() / degPerInch));
        string rightVal = std::to_string(int(rightSide.get_position() / degPerInch));
    }
}

double Chassis::ticksToInches(double ticks)
{
    return (ticks / ticksPerRev) * (odomWheelDiam * M_PI);
}

void Chassis::startOdomLoop()
{
    odomTask = new pros::Task(odomLoop, this, "ODOM");
}

void Chassis::odomLoop(void *p)
{
    static pros::Mutex odomMutex; // local guard so header doesn't need change

    Chassis *ch = static_cast<Chassis *>(p);
    if (ch == nullptr)
        return;
    if (!ch->verticalEnc.has_value() /* || !ch->horizEnc.has_value()*/)
        // ensure vertical encoder has been initialized
        return;
    // ch->horizEnc->set_data_rate(5);
    ch->verticalEnc->set_data_rate(5);

    // seed totals
    double lastH = 0; // ch->ticksToInches(ch->horizEnc->get_position());
    double lastV = ch->ticksToInches(ch->verticalEnc->get_position());
    double lastTheta = ch->inertial.get_rotation() * (M_PI / 180.0);

    if (!std::isfinite(lastV) || !std::isfinite(lastTheta))
        return;

    while (true) // run forever
    {
        // Check if we need to reseed (happens after resetPose)
        // If sensors are reset, lastV and lastTheta will be
        // wildly different than the sensor value leading to large deltas
        if (ch->odomReseedFlag)
        {
            lastH = 0; // ch->ticksToInches(ch->horizEnc->get_position());
            lastV = ch->ticksToInches(ch->verticalEnc->get_position());
            lastTheta = ch->inertial.get_rotation() * (M_PI / 180.0);
            ch->odomReseedFlag = false;
            continue; // Skip this iteration to avoid bad deltas
        }

        // read sensors into locals
        double curH = 0; // ch->ticksToInches(ch->horizEnc->get_position());
        double curV = ch->ticksToInches(ch->verticalEnc->get_position());
        double curTheta = ch->inertial.get_rotation() * (M_PI / 180.0);

        // validate, ensure no division by 0
        if (!std::isfinite(curH) || !std::isfinite(curV) || !std::isfinite(curTheta))
        {
            pros::lcd::set_text(3, "odom: bad sensor read");
            pros::lcd::set_text(4, std::format("v={}", curV));
            pros::lcd::set_text(5, std::format("th={}", curTheta));
            pros::delay(10);
            continue;
        }

        // deltas (local frame)
        double dH = curH - lastH;
        double dV = curV - lastV;
        double deltaTheta = curTheta - lastTheta;

        // normalize deltaTheta to range [-pi, pi] (helps for IMU wrap)
        while (deltaTheta > M_PI)
            deltaTheta -= 2.0 * M_PI;
        while (deltaTheta < -M_PI)
            deltaTheta += 2.0 * M_PI;

        lastH = curH;
        lastV = curV;
        lastTheta = curTheta;

        // compute local-position delta using formula
        // local = 2*sin(dth/2) * (lateralDeltas/dth + lateralOffsets)
        double localLateral = 0.0; // corresponds to horizontal wheel delta
        double localForward = 0.0; // corresponds to vertical wheel delta

        if (fabs(deltaTheta) < 1e-6)
        {
            // small-angle = linear
            localLateral = dH;
            localForward = dV;
        }
        else
        {
            double coef = 2.0 * sin(deltaTheta / 2.0); // chord length / r
            localLateral = coef * (dH / deltaTheta + ch->horizOffset);
            localForward = coef * (dV / deltaTheta + ch->vertOffset);
        }

        // rotate local delta into global using midpoint orientation
        double midTheta = curTheta - deltaTheta / 2.0; // or ch->pose.theta + deltaTheta/2.0
        double cosT = cos(midTheta);
        double sinT = sin(midTheta);
        double globalDy = localForward * cosT - localLateral * sinT;
        double globalDx = localForward * sinT + localLateral * cosT;

        // validate before writing
        if (!std::isfinite(globalDx) || !std::isfinite(globalDy) || fabs(globalDx) > 1e4 || fabs(globalDy) > 1e4)
        {
            continue;
        }

        // atomic pose update
        odomMutex.take();
        ch->pose.x += globalDx;
        ch->pose.y += globalDy;
        ch->pose.theta = curTheta;
        odomMutex.give();

        // delay to avoid overloading brain
        pros::delay(5);
    }
}

Pose Chassis::getPose()
{
    return pose;
}

void Chassis::resetPose(double x, double y, double theta)
{
    pose.x = x;
    pose.y = y;
    pose.theta = theta;

    float oldHeading = inertial.get_heading();
    // reset encoders/inertial
    if (verticalEnc.has_value())
    {
        verticalEnc->reset_position();
    }
    inertial.set_rotation(theta * (180.0 / M_PI));
    float newHeading = inertial.get_heading();

    // Offset g_targetHeading by the heading difference so turns/swings still work
    g_targetHeading += (newHeading - oldHeading);

    // Signal odomLoop to reseed its tracking variables
    odomReseedFlag = true;
}

void Chassis::turnToPoint(double x, double y, float startVel, float maxVel, float endVel)
{
    Pose currentPose = getPose();

    // Calculate the angle to the target point
    double deltaX = x - currentPose.x;
    double deltaY = y - currentPose.y;

    // Calculate target heading in radians, then convert to degrees
    double targetHeadingRad = atan2(deltaY, deltaX);
    float targetHeadingDeg = targetHeadingRad * (180.0 / M_PI);

    // Normalize to 0-360 range
    while (targetHeadingDeg < 0)
        targetHeadingDeg += 360.0;
    while (targetHeadingDeg >= 360.0)
        targetHeadingDeg -= 360.0;

    // get start time
    g_startTime = pros::millis() / 1000.0;

    // calculate total distance to travel
    g_targetHeading = targetHeadingDeg;
    g_totDist = g_targetHeading - inertial.get_heading();
    if (fabs(g_totDist) > 180)
    { // checks for large errors from values being on opposite ends of 360 degrees
        g_totDist = (360.0 - fabs(g_totDist)) * sign(g_totDist * (-1.0));
    }

    // get movement direction
    int moveSign = sign(g_totDist);

    // convert values
    g_startVel = (startVel / 100.0) * g_maxTurnSpeed * moveSign;
    g_maxVel = (maxVel / 100.0) * g_maxTurnSpeed * moveSign;
    g_endVel = (endVel / 100.0) * g_maxTurnSpeed * moveSign;

    initMP(g_totDist, g_startVel, g_maxVel, g_endVel, g_turnMaxAccel * moveSign, g_turnMaxDecel * moveSign);

    // initial conditions
    float startPos = inertial.get_heading();

    float targetDist = 0;
    float targetVel = 0;
    float targetAccel = 0;
    float power = 0;

    float curHeadingError = 0;
    float headingError = 0;
    float totError = 0;
    float prevError = 0;

    g_distTraveled = 0;

    while (true)
    {
        targetDist = getMPdist((pros::millis() / 1000.0) - g_startTime, g_turnMaxAccel * moveSign, g_turnMaxDecel * moveSign);
        targetVel = getMPvel((pros::millis() / 1000.0) - g_startTime, g_turnMaxAccel * moveSign, g_turnMaxDecel * moveSign);
        targetAccel = getMPaccel((pros::millis() / 1000.0) - g_startTime, g_turnMaxAccel * moveSign, g_turnMaxDecel * moveSign);

        curHeadingError = targetDist - (inertial.get_heading() - startPos);
        if (fabs(curHeadingError) > 180)
        {
            curHeadingError = (360.0 - fabs(curHeadingError)) * sign(curHeadingError * (-1.0));
        }

        // power calculations from mp
        power = curHeadingError * turnKP + totError * turnKI + (curHeadingError - prevError) / (dT / 1000.0) * turnKD + targetVel * turnKV + targetAccel * turnKA;

        leftSide.move(power);
        rightSide.move(-power);

        // error assignment for I and D
        totError += curHeadingError;
        prevError = curHeadingError;

        // exit condition calculations
        headingError = g_targetHeading - inertial.get_heading();
        if (fabs(headingError) > 180)
        {
            headingError = (360.0 - fabs(headingError)) * sign(headingError * (-1.0));
        }
        if ((fabs(headingError) <= g_earlyExitRangeTurn) || (pros::millis() / 1000.0 >= g_startTime + g_earlyExitTimeTurn))
        {
            break;
        }

        pros::delay(dT);
    }
    leftSide.set_brake_mode(pros::MotorBrake::hold);
    rightSide.set_brake_mode(pros::MotorBrake::hold);
    leftSide.brake();
    rightSide.brake();
}

void Chassis::moveToPoint(double x, double y, float startVel, float maxVel, float endVel)
{
    g_startTime = pros::millis() / 1000.0;

    Pose startPose = getPose();

    // get initial distance to target
    double deltaX = x - startPose.x;
    double deltaY = y - startPose.y;
    double totalDistance = sqrt(deltaX * deltaX + deltaY * deltaY);

    // get movement direction (forward or backward)
    double initialAngle = atan2(deltaY, deltaX) * (180.0 / M_PI);
    while (initialAngle < 0)
        initialAngle += 360.0;
    while (initialAngle >= 360.0)
        initialAngle -= 360.0;

    double headingDiff = initialAngle - inertial.get_heading();
    if (fabs(headingDiff) > 180.0)
    {
        headingDiff = (360.0 - fabs(headingDiff)) * sign(headingDiff * (-1.0));
    }

    // check if we should go backwards (if target is behind us)
    int moveSign = (fabs(headingDiff) > 90.0) ? -1 : 1;

    // convert velocities
    g_totDist = totalDistance;
    g_startVel = (startVel / 100.0) * g_maxLinSpeed * moveSign;
    g_maxVel = (maxVel / 100.0) * g_maxLinSpeed * moveSign;
    g_endVel = (endVel / 100.0) * g_maxLinSpeed * moveSign;

    initMP(g_totDist, g_startVel, g_maxVel, g_endVel, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);

    float leftStartPos = leftSide.get_position();
    float rightStartPos = rightSide.get_position();
    float trackingStartPos = 0;
    if (verticalEnc.has_value())
    {
        trackingStartPos = verticalEnc->get_position();
    }

    // PID variables
    float leftError = 0, leftTotError = 0, leftPrevError = 0;
    float rightError = 0, rightTotError = 0, rightPrevError = 0;
    float distError = totalDistance;

    while (true)
    {
        // get motion profile targets based on distance traveled
        float timeElapsed = (pros::millis() / 1000.0) - g_startTime;
        float targetDist = getMPdist(timeElapsed, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);
        float targetVel = getMPvel(timeElapsed, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);
        float targetAccel = getMPaccel(timeElapsed, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);
        Pose currentPose = getPose();

        // get current distance to target
        deltaX = x - currentPose.x;
        deltaY = y - currentPose.y;
        double distanceRemaining = sqrt(deltaX * deltaX + deltaY * deltaY);

        // get angle to target
        double targetAngle = atan2(deltaY, deltaX) * (180.0 / M_PI);
        while (targetAngle < 0)
            targetAngle += 360.0;
        while (targetAngle >= 360.0)
            targetAngle -= 360.0;

        // if moving backwards, face opposite direction
        if (moveSign < 0)
        {
            targetAngle += 180.0;
            if (targetAngle >= 360.0)
                targetAngle -= 360.0;
        }

        // get heading error
        float headingError = inertial.get_heading() - targetAngle;
        if (fabs(headingError) > 180.0)
        {
            headingError = (360.0 - fabs(headingError)) * sign(headingError * (-1.0));
        }
        // get lateral correction for heading
        float degDiff = (PI * trackWidth) * (headingError / 360.0) * headingKP;

        // get distance traveled
        if (verticalEnc.has_value())
        {
            g_distTraveled = ticksToInches(verticalEnc->get_position() - trackingStartPos);
        }
        else
        {
            g_distTraveled = (((leftSide.get_position() - leftStartPos) +
                               (rightSide.get_position() - rightStartPos)) /
                              2.0) /
                             degPerInch;
        }
        // get motor errors
        leftError = targetDist - degDiff - ((leftSide.get_position() - leftStartPos) / degPerInch);
        rightError = targetDist + degDiff - ((rightSide.get_position() - rightStartPos) / degPerInch);

        // get motor powers using PID
        float leftPower = leftError * driveKP + leftTotError * driveKI +
                          ((leftError - leftPrevError) / (dT / 1000.0)) * driveKD +
                          targetVel * driveKV + targetAccel * driveKA;
        float rightPower = rightError * driveKP + rightTotError * driveKI +
                           ((rightError - rightPrevError) / (dT / 1000.0)) * driveKD +
                           targetVel * driveKV + targetAccel * driveKA;
        leftSide.move(leftPower);
        rightSide.move(rightPower);

        // update PID terms
        leftTotError += leftError;
        leftPrevError = leftError;
        rightTotError += rightError;
        rightPrevError = rightError;

        // exit conditions: close enough to target or timeout
        distError = totalDistance - g_distTraveled;
        if ((distanceRemaining <= g_earlyExitRangeStr) ||
            (pros::millis() / 1000.0 >= g_startTime + g_totTime + g_earlyExitTimeStr))
        {
            break;
        }

        pros::delay(dT);
    }

    // Stop motors
    leftSide.move(0);
    rightSide.move(0);
}

// move to pose with boomerang controller
void Chassis::moveToPose(double x, double y, double theta, float startVel, float maxVel, float endVel, float lookAheadDistance)
{
    g_startTime = pros::millis() / 1000.0;

    Pose startPose = getPose();

    // get initial distance to target
    double deltaX = x - startPose.x;
    double deltaY = y - startPose.y;
    double totalDistance = sqrt(deltaX * deltaX + deltaY * deltaY);

    // target theta to degrees
    float finalHeadingDeg = theta * (180.0 / M_PI);
    while (finalHeadingDeg < 0)
        finalHeadingDeg += 360.0;
    while (finalHeadingDeg >= 360.0)
        finalHeadingDeg -= 360.0;

    // boomerang controller constants
    const float boomerangKp = 0.015; // proportional gain for steering

    // Determine movement direction (forward or backward)
    double initialAngle = atan2(deltaY, deltaX) * (180.0 / M_PI);
    while (initialAngle < 0)
        initialAngle += 360.0;
    while (initialAngle >= 360.0)
        initialAngle -= 360.0;

    double headingDiff = initialAngle - inertial.get_heading();
    if (fabs(headingDiff) > 180.0)
    {
        headingDiff = (360.0 - fabs(headingDiff)) * sign(headingDiff * (-1.0));
    }

    int moveSign = (fabs(headingDiff) > 90.0) ? -1 : 1;

    g_totDist = totalDistance;
    g_startVel = (startVel / 100.0) * g_maxLinSpeed * moveSign;
    g_maxVel = (maxVel / 100.0) * g_maxLinSpeed * moveSign;
    g_endVel = (endVel / 100.0) * g_maxLinSpeed * moveSign;

    initMP(g_totDist, g_startVel, g_maxVel, g_endVel, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);

    float leftStartPos = leftSide.get_position();
    float rightStartPos = rightSide.get_position();
    float trackingStartPos = 0;
    if (verticalEnc.has_value())
    {
        trackingStartPos = verticalEnc->get_position();
    }

    float leftError = 0, leftTotError = 0, leftPrevError = 0;
    float rightError = 0, rightTotError = 0, rightPrevError = 0;

    // Main control loop
    while (true)
    {
        Pose currentPose = getPose();

        // get current distance to target
        deltaX = x - currentPose.x;
        deltaY = y - currentPose.y;
        double distanceRemaining = sqrt(deltaX * deltaX + deltaY * deltaY);

        // get time elapsed and distance traveled
        float timeElapsed = (pros::millis() / 1000.0) - g_startTime;
        if (verticalEnc.has_value())
        {
            g_distTraveled = ticksToInches(verticalEnc->get_position() - trackingStartPos);
        }
        else
        {
            g_distTraveled = (((leftSide.get_position() - leftStartPos) +
                               (rightSide.get_position() - rightStartPos)) /
                              2.0) /
                             degPerInch;
        }

        // Boomerang Controller
        // get angle to target
        double angleToTarget = atan2(deltaY, deltaX) * (180.0 / M_PI);
        while (angleToTarget < 0)
            angleToTarget += 360.0;
        while (angleToTarget >= 360.0)
            angleToTarget -= 360.0;

        // get carrot point along the line to the final heading
        double carrotAngle = angleToTarget + (finalHeadingDeg - angleToTarget) * (1.0 - fmin(1.0, distanceRemaining / 24.0));

        // normalize carrot angle
        while (carrotAngle < 0)
            carrotAngle += 360.0;
        while (carrotAngle >= 360.0)
            carrotAngle -= 360.0;

        // if moving backwards, reverse the carrot angle
        if (moveSign < 0)
        {
            carrotAngle += 180.0;
            if (carrotAngle >= 360.0)
                carrotAngle -= 360.0;
        }

        // calculate heading error to carrot point
        float headingError = inertial.get_heading() - carrotAngle;
        if (fabs(headingError) > 180.0)
        {
            headingError = (360.0 - fabs(headingError)) * sign(headingError * (-1.0));
        }

        // boomerang steering: proportional to heading error and distance remaining
        float boomerangSteering = headingError * boomerangKp;

        // convert steering angle to differential drive power
        float degDiff = boomerangSteering * trackWidth;

        // get motion profile targets
        float targetDist = getMPdist(timeElapsed, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);
        float targetVel = getMPvel(timeElapsed, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);
        float targetAccel = getMPaccel(timeElapsed, g_linMaxAccel * moveSign, g_linMaxDecel * moveSign);

        // get motor errors
        leftError = targetDist - degDiff - ((leftSide.get_position() - leftStartPos) / degPerInch);
        rightError = targetDist + degDiff - ((rightSide.get_position() - rightStartPos) / degPerInch);

        // get motor powers using PID + feedforward
        float leftPower = leftError * driveKP + leftTotError * driveKI +
                          ((leftError - leftPrevError) / (dT / 1000.0)) * driveKD +
                          targetVel * driveKV + targetAccel * driveKA;
        float rightPower = rightError * driveKP + rightTotError * driveKI +
                           ((rightError - rightPrevError) / (dT / 1000.0)) * driveKD +
                           targetVel * driveKV + targetAccel * driveKA;

        leftSide.move(leftPower);
        rightSide.move(rightPower);

        // update PID terms
        leftTotError += leftError;
        leftPrevError = leftError;
        rightTotError += rightError;
        rightPrevError = rightError;

        // exit condition: close enough to target and facing correct direction
        float finalHeadingError = finalHeadingDeg - inertial.get_heading();
        if (fabs(finalHeadingError) > 180.0)
        {
            finalHeadingError = (360.0 - fabs(finalHeadingError)) * sign(finalHeadingError * (-1.0));
        }

        if ((distanceRemaining <= g_earlyExitRangeStr && fabs(finalHeadingError) <= g_earlyExitRangeTurn) ||
            (pros::millis() / 1000.0 >= g_startTime + g_totTime + g_earlyExitTimeStr))
        {
            break;
        }

        pros::delay(dT);
    }

    leftSide.move(0);
    rightSide.move(0);
}

void Chassis::strEarlyExitConstants(float range, float time)
{
    this->g_earlyExitRangeStr = range;
    this->g_earlyExitTimeStr = time;
}

void Chassis::turnEarlyExitConstants(float range, float time)
{
    this->g_earlyExitRangeTurn = range;
    this->g_earlyExitTimeTurn = time;
}

void Chassis::swingEarlyExitConstants(float range, float time)
{
    this->g_earlyExitRangeSwing = range;
    this->g_earlyExitTimeSwing = time;
}