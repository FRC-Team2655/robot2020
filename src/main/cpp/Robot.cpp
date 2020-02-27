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
#include "commands/DriveDistanceCommand.h"

DriveBaseSubsystem Robot::driveBase;
OI Robot::oi;
ShooterSubsystem Robot::shooter;
IntakeSubsystem Robot::intake;
BeltsSubsystem Robot::belts;

double penc, pgyro;

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
    frc::SmartDashboard::PutNumber("Left P: ", driveBase.kPLeft);
    frc::SmartDashboard::PutNumber("Right P: ", driveBase.kPRight);
    frc::SmartDashboard::PutNumber("Left I: ", driveBase.kILeft);
    frc::SmartDashboard::PutNumber("Right I: ", driveBase.kIRight);
    frc::SmartDashboard::PutNumber("Auto Distance: ", autoDistance);
    frc::SmartDashboard::PutNumber("Encoder Auto P: ", penc);
    frc::SmartDashboard::PutNumber("Gyro Auto P: ", pgyro);


    shooter.setCoastMode();
    belts.setCoastMode();
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
        intake.setLockPID();
    }

    intake.setRollersCoastMode();

    frc::SmartDashboard::PutNumber("Left Current: ", driveBase.leftCurrent());
    frc::SmartDashboard::PutNumber("Right Current: ", driveBase.rightCurrent());

    frc::SmartDashboard::PutNumber("Left Rotations: ", driveBase.getLeftEncoderRotations());
    frc::SmartDashboard::PutNumber("Right Rotations: ", driveBase.getRightEncoderRotations());
    frc::SmartDashboard::PutNumber("Left Rate: ", driveBase.getLeftEncoderRate());
    frc::SmartDashboard::PutNumber("Right Rate: ", driveBase.getRightEncoderRate());
    frc::SmartDashboard::PutNumber("Right SM Rate: ", driveBase.getRightSMRate());
    frc::SmartDashboard::PutNumber("Left SM Rate: ", driveBase.getLeftSMRate());


    frc::SmartDashboard::PutNumber("Top Sensor: ", belts.isProximSensorTopTriggered());
    frc::SmartDashboard::PutNumber("Middle Sensor: ", belts.isProximSensorMiddleTriggered());
    frc::SmartDashboard::PutNumber("Bottom Sensor: ", belts.isProximSensorBottomTriggered());

    frc::SmartDashboard::PutNumber("Gyro Angle: ", driveBase.getIMUAngle());

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
    if (frc::SmartDashboard::GetNumber("Left P: ", 0) != driveBase.kPLeft) {
        driveBase.kPLeft = frc::SmartDashboard::GetNumber("Left P: ", 0);
        driveBase.leftPID.SetP(driveBase.kPLeft);
    }
    if (frc::SmartDashboard::GetNumber("Right P: ", 0) != driveBase.kPRight) {
        driveBase.kPRight = frc::SmartDashboard::GetNumber("Right P: ", 0);
        driveBase.rightPID.SetP(driveBase.kPRight);
    }
    if (frc::SmartDashboard::GetNumber("Left I: ", 0) != driveBase.kILeft) {
        driveBase.kILeft = frc::SmartDashboard::GetNumber("Left I: ", 0);
        driveBase.leftPID.SetI(driveBase.kILeft);
    }
    if (frc::SmartDashboard::GetNumber("Right I: ", 0) != driveBase.kIRight) {
        driveBase.kIRight = frc::SmartDashboard::GetNumber("Right I: ", 0);
        driveBase.rightPID.SetI(driveBase.kIRight);
    }
    if (frc::SmartDashboard::GetNumber("Auto Distance: ", 0) != autoDistance) {
        autoDistance = frc::SmartDashboard::GetNumber("Auto Distance: ", 0);
    }
    if (frc::SmartDashboard::GetNumber("Encoder Auto P: ", 0) != penc) {
        penc = frc::SmartDashboard::GetNumber("Encoder Auto P: ", 0);
    }
    if (frc::SmartDashboard::GetNumber("Gyro Auto P: ", 0) != pgyro) {
        pgyro = frc::SmartDashboard::GetNumber("Gyro Auto P: ", 0);
    }

    frc::SmartDashboard::PutNumber("Intake Arm Current: ", intake.intakeArmCurrent());
    frc::SmartDashboard::PutNumber("Intake Arm Position: ", intake.armPosition());
    frc::SmartDashboard::PutNumber("Intake Arm Percentage: ", intake.intakeMotorValue);

    frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
    driveBase.setBrakeMode();
}

void Robot::DisabledPeriodic() {
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    driveBase.setBrakeMode();
    intake.updateOffset();

    DriveDistanceCommand* autonCmd = new DriveDistanceCommand(autoDistance);
    autonCmd->P_gyro = pgyro;
    autonCmd->P_encoders = penc;
    autonCmd->Schedule();

   /*autonomousCommand = oi.getAutonomousCommand();

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