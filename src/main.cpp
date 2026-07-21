#include "main.h"
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <string>
#include <system_error>
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
#include "pros/rtos.hpp"
#include "chassis.h"

using std::string;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftSide({0, 0}, pros::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::MotorGroup rightSide({0, 0}, pros::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::IMU inertial(0);

// robot measurements
const float wheelDiam = 3.25;
const float wheelCircum = PI * wheelDiam;
const float cartRPM = 600.0;
const float maxDegPerSec = cartRPM * 6;
const float gearRatio = 0.6;
const float degPerInch = 360.0 / (wheelCircum * gearRatio);
const float trackWidth = 10.25;
const float maxSpeed = wheelCircum * cartRPM * gearRatio / 60.0;

Chassis chassis(leftSide, rightSide, inertial, wheelDiam, wheelCircum, cartRPM, maxDegPerSec, gearRatio, degPerInch, trackWidth, maxSpeed);

// driver control values
const int deadband = 8;

int sign(int x)
{
  return (x < 0) ? (-1.0) : 1.0;
}

// auton selector
int currAuton = 1;

void incAuto()
{
  currAuton++;
  if (currAuton == 6)
  {
    currAuton = 0;
  }
}

void initialize()
{
  pros::lcd::initialize();
  inertial.reset(true);
  controller.rumble(".");
  // UNCOMMENT TO ENABLE ODOM (requires tracking wheel)
  // chassis.startOdomLoop();
}

void disabled()
{
  // . . .
}

void competition_initialize()
{
  pros::lcd::register_btn1_cb(incAuto);
  pros::lcd::set_text(4, "comp init");
  while (true)
  {
    if (controller.get_digital_new_press(DIGITAL_DOWN))
    {
      incAuto();
    }
    switch (currAuton)
    {
    case 0:
      pros::lcd::set_text(2, "auto 0");
      break;
    case 1:
      pros::lcd::set_text(2, "auto 1");
      break;
    case 2:
      pros::lcd::set_text(2, "auto 2");
      break;
    case 3:
      pros::lcd::set_text(2, "auto 4");
      break;
    case 4:
      pros::lcd::set_text(2, "auto 5");
      break;
    case 5:
      pros::lcd::set_text(2, "auto 6");
      break;
    }
    pros::delay(2);
  }
}

/* AUTOS */

void auto0()
{
  pros::lcd::set_text(3, "auto 0");
}

void auto1()
{
  pros::lcd::set_text(3, "auto 1");
}
void auto2()
{
  pros::lcd::set_text(3, "auto 2");
}
void auto3()
{
  pros::lcd::set_text(3, "auto 3");
}

void auto4()
{
  pros::lcd::set_text(3, "auto 4");
}

void auto5()
{
  pros::lcd::set_text(3, "auto 5");
}

void autonomous()
{
  switch (currAuton)
  {
  case 0:
    auto0();
    break;
  case 1:
    auto1();
    break;
  case 2:
    auto2();
    break;
  case 3:
    auto3();
    break;
  case 4:
    auto4();
    break;
  case 5:
    auto5();
    break;
  default:
    auto0();
  }
}

void arcade(float throttle, float turn)
{
  float leftPower = throttle + turn;
  float rightPower = throttle - turn;

  leftSide.move(leftPower);
  rightSide.move(rightPower);
}

static int ease(int in)
{
  const double maxv = 127.0; // controller analog maximum magnitude
  if (in == 0)
    return 0;
  double normalized = static_cast<double>(in) / maxv;
  double eased = 1 - sqrt(1 - pow(normalized, 2));
  int out = static_cast<int>(std::round(eased * maxv));
  if (out > maxv)
    out = maxv;
  if (out < -maxv)
    out = -maxv;
  return out;
}

void opcontrol()
{
  while (true)
  {

    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // cubic easing
    leftY = ease(leftY) * sign(leftY);
    rightX = ease(rightX) * sign(rightX);
    arcade(leftY, rightX);

    pros::delay(10);
  }
}