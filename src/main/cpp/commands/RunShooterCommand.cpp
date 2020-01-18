#include "commands/RunShooterCommand.h"
#include "Robot.h"

RunShooterCommand::RunShooterCommand(double speed) : speed(speed) {
  //AddRequirements(&Robot::shooter);
}

void RunShooterCommand::Initialize() {
  std::cout << "Initialize" << std::endl;
}

void RunShooterCommand::Execute() {
  std::cout << "execute" << std::endl;
  Robot::shooter.runShooter(speed);
}

void RunShooterCommand::End(bool interrupted) {
  Robot::shooter.stopShooter();
}

bool RunShooterCommand::IsFinished() { return false; }
