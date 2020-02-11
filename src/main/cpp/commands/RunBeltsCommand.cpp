/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RunBeltsCommand.h"
#include "Robot.h"

RunBeltsCommand::RunBeltsCommand(double speed) : speed(speed) {
}

// Called when the command is initially scheduled.
void RunBeltsCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunBeltsCommand::Execute() {
  Robot::shooter.runBelts(speed);
}

// Called once the command ends or is interrupted.
void RunBeltsCommand::End(bool interrupted) {
  Robot::shooter.stopBelts();
}

// Returns true when the command should end.
bool RunBeltsCommand::IsFinished() { return false; }
