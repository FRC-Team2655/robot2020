#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "RobotMap.h"
#include <ctre/Phoenix.h>
#include <frc/Timer.h>

#include <frc/DigitalInput.h>

using MotorType = rev::CANSparkMax::MotorType;

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  void runShooterPercentage(double speed);
  void runShooterVelocity();
  void setCoastMode();
  void stopShooter();

  double getShooter1Current();
  double getShooter1AccumError();

  bool isProximSensor1Triggered();

  void runBelts(double speed, bool useProximitySensor);
  void stopBelts();

  double getRPM();

  void Periodic();

  rev::CANPIDController shooter1PID = shooter1.GetPIDController();

 private:
  rev::CANSparkMax shooter1 {Shooter1ID, MotorType::kBrushless};
  rev::CANSparkMax shooter2 {Shooter2ID, MotorType::kBrushless};

  rev::CANEncoder shooterEncoder1 = shooter1.GetEncoder();

  frc::DigitalInput proximSensor1 {proximitySensor1Channel};

  WPI_VictorSPX kicker {KickerID};

  WPI_TalonSRX leftBelt {BeltLeft};
  WPI_TalonSRX rightBelt {BeltRight};
  WPI_VictorSPX bottomBelt {BeltBottom};

  double shooterSpeed;

  double kP = 0.0000001; 
  double kI = 0; 
  double kD = 0; 
  double kFF = 0.000176;
  double kIz = 0;
  double kMax = 1;
  double kMin = 0;
};