#pragma once

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Button.h>

#include "RobotMap.h"
#include "team2655/joystick.hpp"
#include "commands/RunShooterCommand.h"
#include "subsystems/ShooterSubsystem.h"

#include <frc/Joystick.h>

#include <frc2/command/InstantCommand.h>

using namespace team2655;

class OI {
public:
  OI();
  frc::Joystick *js0;
  frc2::JoystickButton *xBtn;  

  // Configurations for the joystick deadband and cubic function.
  jshelper::AxisConfig driveAxisConfig = jshelper::createAxisConfig(.1, 0, .5);
  jshelper::AxisConfig rotateAxisConfig = jshelper::createAxisConfig(0.1);

  RunShooterCommand rsCommand {0};
  
  void runButtons();
  frc2::Command* getAutonomousCommand();
};