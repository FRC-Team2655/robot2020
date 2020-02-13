/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/MoveIntakeArmCommand.h"
#include "Robot.h"

MoveIntakeArmCommand::MoveIntakeArmCommand(double position) : position(position) {
  AddRequirements(&Robot::intake);
}

// Called when the command is initially scheduled.
void MoveIntakeArmCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MoveIntakeArmCommand::Execute() {
  Robot::intake.moveArm(position);
}

// Called once the command ends or is interrupted.
void MoveIntakeArmCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool MoveIntakeArmCommand::IsFinished() { return false; }
