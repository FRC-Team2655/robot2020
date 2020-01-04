/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/buttons/JoystickButton.h>
#include <commands/DriveJoystickCommand.h>
#include <RobotMap.h>
#include <frc/Joystick.h>
#include <team2655/joystick.hpp>

using namespace team2655;

class OI {
public:
  OI();
  frc::Joystick *js0;

  // Configurations for the joystick deadband and cubic function.
  jshelper::AxisConfig driveAxisConfig = jshelper::createAxisConfig(.1, 0, .5);
  jshelper::AxisConfig rotateAxisConfig = jshelper::createAxisConfig(0.1);
};
