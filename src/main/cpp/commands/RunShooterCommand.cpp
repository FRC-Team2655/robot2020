#include "commands/RunShooterCommand.h"
#include "Robot.h"

RunShooterCommand::RunShooterCommand(double shooterSpeed, double beltSpeed) : shooterSpeed(shooterSpeed),
                                                                              beltSpeed(beltSpeed)  {
  AddRequirements(&Robot::shooter);
}

void RunShooterCommand::Initialize() {
}

void RunShooterCommand::Execute() {
  Robot::shooter.runShooter(shooterSpeed);
  Robot::shooter.runBelts(beltSpeed);
}

void RunShooterCommand::End(bool interrupted) {
  Robot::shooter.stopShooter();
  Robot::shooter.stopBelts();
}

bool RunShooterCommand::IsFinished() { return false; }
