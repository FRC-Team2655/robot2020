/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RotateToGyroAngleCommand.h"
#include <Robot.h>

RotateToGyroAngleCommand::RotateToGyroAngleCommand(double TargetAngle) : TargetAngle(TargetAngle){
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RotateToGyroAngleCommand::Initialize() {
  

}

// Called repeatedly when this Command is scheduled to run
void RotateToGyroAngleCommand::Execute() {}

// Called once the command ends or is interrupted.
void RotateToGyroAngleCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool RotateToGyroAngleCommand::IsFinished() { return false; }
