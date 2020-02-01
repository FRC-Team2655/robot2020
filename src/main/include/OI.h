#pragma once

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Button.h>

#include "RobotMap.h"
#include "team2655/joystick.hpp"
#include "commands/RunShooterCommand.h"
#include "subsystems/ShooterSubsystem.h"

#include <frc/Joystick.h>

using namespace team2655;

class OI {
public:
  OI();
  frc::Joystick *js0;
  frc2::JoystickButton *xBtn;
  frc2::JoystickButton *squareBtn;  

  // Configurations for the joystick deadband and cubic function.
  jshelper::AxisConfig driveAxisConfig = jshelper::createAxisConfig(.1, 0, .5);
  jshelper::AxisConfig rotateAxisConfig = jshelper::createAxisConfig(0.1);

  RunShooterCommand riCommand {0, 0.5};
  RunShooterCommand rsCommand {0, 0.5};
  
  void runButtons();
};