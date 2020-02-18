/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeSubsystem.h"
#include "Robot.h"

using NeutralMode = ctre::phoenix::motorcontrol::NeutralMode;
using IdleMode = rev::CANSparkMax::IdleMode;

IntakeSubsystem::IntakeSubsystem() {
    std::cout << intakePositionOffset << std::endl;

    intakeArm.SetSmartCurrentLimit(15);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::runRollers(double speed) {
    intakeRollers.Set(speed);
}

void IntakeSubsystem::stopRollers() {
    intakeRollers.Set(0);
}

void IntakeSubsystem::moveArmIn() {
    intakeMotorValue = intakeInPID.Calculate(armPosition());

    intakeArm.Set(intakeMotorValue);
}

void IntakeSubsystem::moveArmOut() {
    intakeMotorValue = intakeOutPID.Calculate(armPosition());

    intakeArm.Set(intakeMotorValue);
}

double IntakeSubsystem::armRawPosition() {
    return (double)(intakeEnc.Get());
} 

double IntakeSubsystem::armPosition() {
    return (armRawPosition() - intakePositionOffset);
}

void IntakeSubsystem::setRollersCoastMode() {
    intakeRollers.SetNeutralMode(NeutralMode::Coast);
}

void IntakeSubsystem::setArmBrakeMode() {
    intakeArm.SetIdleMode(IdleMode::kBrake);
}

double IntakeSubsystem::intakeArmCurrent() {
    return intakeArm.GetOutputCurrent();
}

void IntakeSubsystem::stopArm() {
    intakeArm.Set(0);
}

void IntakeSubsystem::resetArmEnc() {
    intakeEnc.Reset();
}

void IntakeSubsystem::updateOffset() {
    intakePositionOffset = armRawPosition();
    std::cout << intakePositionOffset << std::endl;
}

void IntakeSubsystem::setCurrent15() {
    intakeArm.SetSmartCurrentLimit(15);
}

void IntakeSubsystem::setCurrent40() {
    intakeArm.SetSmartCurrentLimit(40);
}

void IntakeSubsystem::setLockPID() {
    intakeMotorValue = intakeLockPID.Calculate(armPosition());

    intakeArm.Set(intakeMotorValue);
}

void IntakeSubsystem::setInPID() {
    intakeMotorValue = intakeInPID.Calculate(armPosition());

    intakeArm.Set(intakeMotorValue);
}