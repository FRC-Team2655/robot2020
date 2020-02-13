/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() {}

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