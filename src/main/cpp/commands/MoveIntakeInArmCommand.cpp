/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/MoveIntakeInArmCommand.h"
#include "Robot.h"

MoveIntakeInArmCommand::MoveIntakeInArmCommand(double position) : position(position) {
  AddRequirements(&Robot::intake);
}

// Called when the command is initially scheduled.
void MoveIntakeInArmCommand::Initialize() {
  Robot::intake.setCurrent40();

  Robot::intake.intakeInPID.SetSetpoint(position);
  Robot::intake.intakeInPID.SetTolerance(armTolerance);
}

// Called repeatedly when this Command is scheduled to run
void MoveIntakeInArmCommand::Execute() {
  Robot::intake.moveArmIn();
}

// Called once the command ends or is interrupted.
void MoveIntakeInArmCommand::End(bool interrupted) {
  Robot::intake.stopArm();

  Robot::intake.setCurrent15();

  Robot::intake.isIntakeOut = false;
  Robot::intake.isIntakeLocked = true;
}

// Returns true when the command should end.
bool MoveIntakeInArmCommand::IsFinished() { 
  return Robot::intake.intakeInPID.AtSetpoint();
}
