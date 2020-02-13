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

  void moveArm(double position);

  double armPosition();

  void Periodic();

private:
  WPI_TalonSRX intakeRollers {RollerShooters};

  rev::CANSparkMax intakeArm {IntakeArm, MotorType::kBrushless};
  frc::DutyCycleEncoder intakeEnc {IntakePWM};
  frc2::PIDController intakePID {5e-4, 0, 0};
};
