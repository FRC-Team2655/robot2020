/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/RunCommand.h>

DriveBaseSubsystem Robot::driveBase;
OI Robot::oi;
ShooterSubsystem Robot::shooter;
IntakeSubsystem Robot::intake;

void Robot::RobotInit() {
    frc::SmartDashboard::PutBoolean("Reset Encoders", false);

    frc::SmartDashboard::PutNumber("Intake In P: ", kInP);
    frc::SmartDashboard::PutNumber("Intake In I: ", kInI);
    frc::SmartDashboard::PutNumber("Intake In D: ", kInD);
    frc::SmartDashboard::PutNumber("Intake Out P: ", kOutP);
    frc::SmartDashboard::PutNumber("Intake Out I: ", kOutI);
    frc::SmartDashboard::PutNumber("Intake Out D: ", kOutD);
    frc::SmartDashboard::PutNumber("Intake Lock P: ", kLockP);
    frc::SmartDashboard::PutNumber("Intake Lock I: ", kLockI);
    frc::SmartDashboard::PutNumber("Intake Lock D: ", kLockD);

    shooter.setCoastMode();
    driveBase.setCoastMode();
    intake.setArmBrakeMode();
    intake.setRollersCoastMode();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { 
    if (intake.isIntakeLocked) {
        if (intake.armPosition() >= (intakeInPosition - 0.006)) {
            intake.setLockPID();
            std::cout << "Lock PID" << std::endl;
        }else{
            intake.setInPID();
            std::cout << "In PID" << std::endl;
        }
    }

    intake.setRollersCoastMode();
    frc::SmartDashboard::PutNumber("Left Output: ", driveBase.getLeftEncoderOutput());
    frc::SmartDashboard::PutNumber("Right Output: ", driveBase.getRightEncoderOutput());

    frc::SmartDashboard::PutNumber("Left Current: ", driveBase.leftCurrent());
    frc::SmartDashboard::PutNumber("Right Current: ", driveBase.rightCurrent());

    if (frc::SmartDashboard::GetBoolean("Reset Encoders", false) == true) {
        driveBase.resetEncoders();
        frc::SmartDashboard::PutBoolean("Reset Encoders", false);
    }
    if (frc::SmartDashboard::GetNumber("Intake In P: ", 0) != kInP) {
        kInP = frc::SmartDashboard::GetNumber("Intake In P: ", 0);
        intake.intakeInPID.SetP(kInP);
    }
    if (frc::SmartDashboard::GetNumber("Intake In I: ", 0) != kInI) {
        kInI = frc::SmartDashboard::GetNumber("Intake In I: ", 0);
        intake.intakeInPID.SetI(kInI);
    }
    if (frc::SmartDashboard::GetNumber("Intake In D: ", 0) != kInD) {
        kInD = frc::SmartDashboard::GetNumber("Intake In D: ", 0);
        intake.intakeInPID.SetD(kInD);
    }
    if (frc::SmartDashboard::GetNumber("Intake Out D: ", 0) != kOutD) {
        kOutD = frc::SmartDashboard::GetNumber("Intake Out D: ", 0);
        intake.intakeOutPID.SetD(kOutD);
    }
    if (frc::SmartDashboard::GetNumber("Intake Out P: ", 0) != kOutP) {
        kOutP = frc::SmartDashboard::GetNumber("Intake Out P: ", 0);
        intake.intakeOutPID.SetP(kOutP);
    }
    if (frc::SmartDashboard::GetNumber("Intake Out I: ", 0) != kOutI) {
        kOutI = frc::SmartDashboard::GetNumber("Intake Out I: ", 0);
        intake.intakeOutPID.SetI(kOutI);
    }
    if (frc::SmartDashboard::GetNumber("Intake Velocity: ", 0) != intake.kVelocity_) {
        intake.kVelocity_ = frc::SmartDashboard::GetNumber("Intake Velocity: ", intake.kVelocity_);
    }
    if (frc::SmartDashboard::GetNumber("Intake Lock P: ", 0) != kLockP) {
        kLockP = frc::SmartDashboard::GetNumber("Intake Lock P: ", 0);
        intake.intakeLockPID.SetP(kLockP);
    }
    if (frc::SmartDashboard::GetNumber("Intake Lock I: ", 0) != kLockI) {
        kLockI = frc::SmartDashboard::GetNumber("Intake Lock I: ", 0);
        intake.intakeLockPID.SetI(kLockI);
    }
    if (frc::SmartDashboard::GetNumber("Intake Lock D: ", 0) != kLockD) {
        kLockD = frc::SmartDashboard::GetNumber("Intake Lock D: ", 0);
        intake.intakeLockPID.SetD(kLockD);
    }

    frc::SmartDashboard::PutNumber("Intake Arm Current: ", intake.intakeArmCurrent());
    frc::SmartDashboard::PutNumber("Intake Arm Position: ", intake.armPosition());
    frc::SmartDashboard::PutNumber("Intake Arm Percentage: ", intake.intakeMotorValue);
    frc::SmartDashboard::PutNumber("Intake Setpoint: ", intake.intakeInPID.GetSetpoint());

    frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
    driveBase.setCoastMode();
}

void Robot::DisabledPeriodic() {
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    driveBase.setCoastMode();
    intake.updateOffset();

   /* autonomousCommand = oi.getAutonomousCommand();

    if (autonomousCommand != nullptr) {
        autonomousCommand->Schedule();
    }*/
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
    /*if (autonomousCommand != nullptr) {
        autonomousCommand->Cancel();
        autonomousCommand = nullptr;
    }*/

    driveBase.setCoastMode();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif