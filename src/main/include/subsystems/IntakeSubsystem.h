/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "RobotMap.h"
#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>

using MotorType = rev::CANSparkMax::MotorType;

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  void setCoastMode();
  void runIntakeRollers(double startingSpeed);
  void stopIntakeRollers();

  void moveIntake(double position);

  void Periodic();

 private:
  double intakeSpeed;

  rev::CANSparkMax intakeRollers {rollersIntake, MotorType::kBrushless};
  rev::CANSparkMax intake {intakeMotor, MotorType::kBrushless};

  rev::CANEncoder intakeEnc = intake.GetEncoder();
  rev::CANPIDController intakePID = intake.GetPIDController();
};
