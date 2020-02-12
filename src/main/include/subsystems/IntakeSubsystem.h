/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include "RobotMap.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();
  void runRollers(double speed);
  void stopRollers();

  void Periodic();

private:
  WPI_TalonSRX intakeRollers {RollerShooters};
};
