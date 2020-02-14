/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeSubsystem.h"

using NeutralMode = ctre::phoenix::motorcontrol::NeutralMode;
using IdleMode = rev::CANSparkMax::IdleMode;

IntakeSubsystem::IntakeSubsystem() {
    intakeArm.SetSmartCurrentLimit(20);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::runRollers(double speed) {
    intakeRollers.Set(speed);
}

void IntakeSubsystem::stopRollers() {
    intakeRollers.Set(0);
}

void IntakeSubsystem::moveArm(double position) {
    intakeArm.Set(intakePID.Calculate(armPosition(), position));
}

double IntakeSubsystem::armPosition() {
    return (double)(intakeEnc.Get());
} 

void IntakeSubsystem::setRollersCoastMode() {
    intakeRollers.SetNeutralMode(NeutralMode::Coast);
}

void IntakeSubsystem::setArmBrakeMode() {
    intakeArm.SetIdleMode(IdleMode::kBrake);
}