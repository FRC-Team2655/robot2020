/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include "RobotMap.h"
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>

using MotorType = rev::CANSparkMax::MotorType;

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();
  void runRollers(double speed);
  void stopRollers();
  void stopArm();
  void setRollersCoastMode();
  void setArmBrakeMode();

  void moveArmIn();
  void moveArmOut();
  double armRawPosition();
  double armPosition();

  void updateOffset();

  void setCurrent15();
  void setCurrent40();

  void setLockPID();
  void setInPID();

  double intakeArmCurrent();

  void resetArmEnc();

  void Periodic();

  double kVelocity_;
  frc2::PIDController intakeInPID {2.0, 0, 0.1};
  frc2::PIDController intakeOutPID {0.85, 0, 0.001};
  frc2::PIDController intakeLockPID {0.000001, 0.000001, 0};
  
  double intakePositionOffset;
  double intakeMotorValue = 0;

  bool isIntakeOut = false;
  bool isIntakeLocked = true;
private:
  WPI_TalonSRX intakeRollers {RollerShooters};
  rev::CANSparkMax intakeArm {IntakeArm, MotorType::kBrushless};

  frc::DutyCycleEncoder intakeEnc {IntakePWM};
};
