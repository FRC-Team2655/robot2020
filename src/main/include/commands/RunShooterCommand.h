#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class RunShooterCommand
    : public frc2::CommandHelper<frc2::CommandBase, RunShooterCommand> {
 public:
  RunShooterCommand(double speed, double beltSpeed);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
  double speed;
  double beltSpeed;
};
