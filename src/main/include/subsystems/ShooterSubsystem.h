#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "RobotMap.h"
#include "commands/RunShooterCommand.h"

using MotorType = rev::CANSparkMax::MotorType;

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  void runShooter(double speed);
  void setCoastMode();
  void stopShooter();

  void Periodic();

 private:
  rev::CANSparkMax shooterMaster {ShooterMaster, MotorType::kBrushless};
  rev::CANSparkMax shooterSlave1 {ShooterSlave1, MotorType::kBrushless};
  rev::CANSparkMax shooterSlave2 {ShooterSlave2, MotorType::kBrushless};

  rev::CANSparkMax kicker {KickerID, MotorType::kBrushless};

  rev::CANSparkMax beltForward {ForwardBelt, MotorType::kBrushless};
  rev::CANSparkMax beltBackward {BackwardBelt, MotorType::kBrushless};
  rev::CANSparkMax beltBottom {BottomBelt, MotorType::kBrushless};

  double shooterSpeed;
};
