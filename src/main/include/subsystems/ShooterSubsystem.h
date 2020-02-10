#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "RobotMap.h"
#include "commands/RunShooterCommand.h"
#include <ctre/Phoenix.h>

using MotorType = rev::CANSparkMax::MotorType;

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  void runShooter(double speed);
  void setCoastMode();
  void stopShooter();

  void runBelts(double speed);
  void stopBelts();

  void Periodic();

 private:
  rev::CANSparkMax shooter1 {Shooter1ID, MotorType::kBrushless};
  rev::CANSparkMax shooter2 {Shooter2ID, MotorType::kBrushless};

  WPI_TalonSRX kicker {KickerID};

  WPI_TalonSRX leftBelt {BeltLeft};
  WPI_TalonSRX rightBelt {BeltRight};
  WPI_TalonSRX bottomBelt {BeltBottom};

  double shooterSpeed;
};
