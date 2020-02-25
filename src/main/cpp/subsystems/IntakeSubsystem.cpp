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

    intakeArm.SetSmartCurrentLimit(armRestCurrent);
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
    static double lastArmVal = 0;
    double read0, read1;
    int count = 0;
    read0 = 0;
    read1 = 1;
    while(count < 10 && read0 != read1)
    {
        read0 = intakeEnc.GetOutput();
        count++;
        read1 = intakeEnc.GetOutput();
    }
    if(count < 10)
    {
        //update arm value if they were equal
        lastArmVal = read0;
    }

    //invert output
    return -1 * lastArmVal;
} 

double IntakeSubsystem::armPosition() {
    return (double)(armRawPosition() - intakePositionOffset);
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

void IntakeSubsystem::updateOffset() {
    intakePositionOffset = armRawPosition();
    std::cout << intakePositionOffset << std::endl;
}

void IntakeSubsystem::setCurrent(double current) {
    intakeArm.SetSmartCurrentLimit(current);
}

void IntakeSubsystem::setCurrent40() {
    intakeArm.SetSmartCurrentLimit(40);
}

void IntakeSubsystem::setLockPID() {
    intakeLockPID.SetSetpoint(intakeInPosition);

    intakeMotorValue = intakeLockPID.Calculate(armPosition());

    intakeArm.Set(intakeMotorValue);
}

void IntakeSubsystem::setInPID() {
    intakeLockPID.SetSetpoint(intakeInPosition);

    intakeMotorValue = intakeInPID.Calculate(armPosition());

    intakeArm.Set(intakeMotorValue);
}

void IntakeSubsystem::moveArmManual() {
    intakeArm.Set(0.2);
}

void IntakeSubsystem::resetArmEnc() {
    //intakeEnc.Reset();
}