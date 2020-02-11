#include "commands/RunShooterCommand.h"
#include "Robot.h"

RunShooterCommand::RunShooterCommand(double speed) : speed(speed){
}

void RunShooterCommand::Initialize() {
}

void RunShooterCommand::Execute() {
  Robot::shooter.runShooter(speed);
}

void RunShooterCommand::End(bool interrupted) {
  Robot::shooter.stopShooter();
}

bool RunShooterCommand::IsFinished() { return false; }
