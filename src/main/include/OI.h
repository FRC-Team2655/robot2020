#pragma once

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Button.h>

#include "RobotMap.h"
#include "team2655/joystick.hpp"

#include <frc/Joystick.h>

using namespace team2655;

class OI {
public:
  OI();
  frc::Joystick *js0;

  // Configurations for the joystick deadband and cubic function.
  jshelper::AxisConfig driveAxisConfig = jshelper::createAxisConfig(.1, 0, .5);
  jshelper::AxisConfig rotateAxisConfig = jshelper::createAxisConfig(0.1);
};