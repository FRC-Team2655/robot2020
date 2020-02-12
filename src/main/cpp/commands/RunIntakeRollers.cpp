/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RunIntakeRollers.h"
#include "Robot.h"

RunIntakeRollers::RunIntakeRollers(double speed) : speed(speed) {
  AddRequirements(&Robot::intake);
}

// Called when the command is initially scheduled.
void RunIntakeRollers::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunIntakeRollers::Execute() {
  Robot::intake.runRollers(speed);
}

// Called once the command ends or is interrupted.
void RunIntakeRollers::End(bool interrupted) {
  Robot::intake.stopRollers();
}

// Returns true when the command should end.
bool RunIntakeRollers::IsFinished() { return false; }
