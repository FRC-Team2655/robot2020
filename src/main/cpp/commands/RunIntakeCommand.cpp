/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RunIntakeCommand.h"
#include "Robot.h"

RunIntakeCommand::RunIntakeCommand(double speed) : speed(speed) {
  AddRequirements(&Robot::intake);
}

void RunIntakeCommand::Initialize() {
}

void RunIntakeCommand::Execute() {
  Robot::intake.runIntakeRollers(speed);
}

void RunIntakeCommand::End(bool interrupted) {
  Robot::intake.stopIntakeRollers();
}

bool RunIntakeCommand::IsFinished() { return false; }
