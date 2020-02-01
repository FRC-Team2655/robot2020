/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeSubsystem.h"

using IdleMode = rev::CANSparkMax::IdleMode;

IntakeSubsystem::IntakeSubsystem() {
  intakePID.SetP(1e-4);
  intakePID.SetI(0);
  intakePID.SetD(0);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::setCoastMode() {
    intake.SetIdleMode(IdleMode::kCoast);
}

void IntakeSubsystem::runIntakeRollers(double startingSpeed) {
    if (intakeSpeed < startingSpeed) {
        intakeSpeed = startingSpeed;
    }else{
        intakeSpeed += incrementIntakeSpeed;
    }

    if (intakeSpeed >= maxIntakeSpeed) {
        intakeSpeed = maxIntakeSpeed;
    }

    intake.Set(intakeSpeed);
}

void IntakeSubsystem::stopIntakeRollers() {
    intakeSpeed = 0;
    intake.Set(0);
}

void IntakeSubsystem::moveIntake(double position) {
    intakePID.SetReference(position, rev::ControlType::kPosition);
}