#include "commands/RunShooterCommand.h"
#include "Robot.h"

RunShooterCommand::RunShooterCommand(double speed, double beltSpeed) : speed(speed), 
                                                                        beltSpeed(beltSpeed) {
  AddRequirements(&Robot::shooter);
}

void RunShooterCommand::Initialize() {
}

void RunShooterCommand::Execute() {
  Robot::shooter.runShooter(speed);
  Robot::shooter.runBelts(beltSpeed);
}

void RunShooterCommand::End(bool interrupted) {
  Robot::shooter.stopShooter();
  Robot::shooter.stopBelts();
}

bool RunShooterCommand::IsFinished() { return false; }
