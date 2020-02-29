/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include "frc/Timer.h"

#include "subsystems/DriveBaseSubsystem.h"
#include "OI.h"
#include "commands/DriveJoystickCommand.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/BeltsSubsystem.h"
#include "subsystems/LEDSubsystem.h"

class Robot : public frc::TimedRobot {
 public:
  static OI oi;
  static DriveBaseSubsystem driveBase;
  static ShooterSubsystem shooter;
  static IntakeSubsystem intake;
  static BeltsSubsystem belts;
  static LEDSubsystem leds;

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  double autoDistance;
private:
  frc2::Command* autonomousCommand = nullptr;

  double kInP = 1.0; 
  double kInI = 0.0; 
  double kInD = 0.1;
  double kOutP = 0.85;
  double kOutI = 0.0;
  double kOutD = 0.001;
  double kLockP = 1.0;
  double kLockI = 0.001;
  double kLockD = 0.0;
};
