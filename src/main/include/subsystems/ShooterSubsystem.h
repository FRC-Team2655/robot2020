#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "RobotMap.h"
#include "commands/RunShooterPercentageCommand.h"
#include <ctre/Phoenix.h>
#include <frc/Timer.h>

using MotorType = rev::CANSparkMax::MotorType;

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  void runShooterPercentage(double startingSpeed);
  void runShooterVelocity();
  void setCoastMode();
  void stopShooter();

  void runBelts(double speed);
  void stopBelts();

  double getRPM();
  double getShooter1Current();
  double getShooter2Current();
  double getShooter1AccumError();
  double getShooter2AccumError();

  void Periodic();

  rev::CANPIDController shooter1PID = shooter1.GetPIDController();
  rev::CANPIDController shooter2PID = shooter2.GetPIDController();

  double kVelocity_ = 5200;

 private:
  rev::CANSparkMax shooter1 {Shooter1ID, MotorType::kBrushless};
  rev::CANSparkMax shooter2 {Shooter2ID, MotorType::kBrushless};

  rev::CANEncoder shooterEncoder1 = shooter1.GetEncoder();
  rev::CANEncoder shooterEncoder2 = shooter2.GetEncoder();

  WPI_VictorSPX kicker {KickerID};

  WPI_TalonSRX leftBelt {BeltLeft};
  WPI_TalonSRX rightBelt {BeltRight};
  WPI_VictorSPX bottomBelt {BeltBottom};

  double shooterSpeed;
};
