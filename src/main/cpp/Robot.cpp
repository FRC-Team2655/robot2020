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

    frc::SmartDashboard::PutNumber("Shooter P: ", kP);
    frc::SmartDashboard::PutNumber("Shooter I: ", kI);
    frc::SmartDashboard::PutNumber("Shooter D: ", kD);
    frc::SmartDashboard::PutNumber("Shooter Iz: ", kIz);
    frc::SmartDashboard::PutNumber("Shooter FF: ", kFF);
    frc::SmartDashboard::PutNumber("Shooter Max: ", kMax);
    frc::SmartDashboard::PutNumber("Shooter Min: ", kMin);
    frc::SmartDashboard::PutNumber("Shooter Velocity: ", shooter.kVelocity_);

    shooter.setCoastMode();
    driveBase.setCoastMode();
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
    frc::SmartDashboard::PutNumber("Left Output: ", driveBase.getLeftEncoderOutput());
    frc::SmartDashboard::PutNumber("Right Output: ", driveBase.getRightEncoderOutput());
    frc::SmartDashboard::PutNumber("Shooter RPM: ", shooter.getRPM());

    if (frc::SmartDashboard::GetBoolean("Reset Encoders", false) == true) {
        driveBase.resetEncoders();
        frc::SmartDashboard::PutBoolean("Reset Encoders", false);
    }

    if (frc::SmartDashboard::GetNumber("Shooter P: ", 0) != kP) {
        kP = frc::SmartDashboard::GetNumber("Shooter P: ", 0);
        shooter.shooter1PID.SetP(kP);
        shooter.shooter2PID.SetP(kP);
    }
    if (frc::SmartDashboard::GetNumber("Shooter I: ", 0) != kI) {
        kI = frc::SmartDashboard::GetNumber("Shooter I: ", 0);
        shooter.shooter1PID.SetI(kI);
        shooter.shooter2PID.SetI(kI);
    }
    if (frc::SmartDashboard::GetNumber("Shooter D: ", 0) != kD) {
        kD = frc::SmartDashboard::GetNumber("Shooter D: ", 0);
        shooter.shooter1PID.SetD(kD);
        shooter.shooter2PID.SetD(kD);
    }
    if (frc::SmartDashboard::GetNumber("Shooter FF: ", 0) != kFF) {
        kFF = frc::SmartDashboard::GetNumber("Shooter FF: ", 0);
        shooter.shooter1PID.SetFF(kFF);
        shooter.shooter2PID.SetFF(kFF);
    }
    if (frc::SmartDashboard::GetNumber("Shooter Iz: ", 0) != kIz) {
        kIz = frc::SmartDashboard::GetNumber("Shooter Iz: ", 0);
        shooter.shooter1PID.SetIZone(kIz);
        shooter.shooter2PID.SetIZone(kIz);
    }
    if ((frc::SmartDashboard::GetNumber("Shooter Max: ", 0) != kMax) || (frc::SmartDashboard::GetNumber("Shooter Min: ", 0 != kMin))) {
        kMax = frc::SmartDashboard::GetNumber("Shooter Max: ", 0);
        kMin = frc::SmartDashboard::GetNumber("Shooter Min: ", 0);
        shooter.shooter1PID.SetOutputRange(kMin, kMax);
        shooter.shooter2PID.SetOutputRange(kMin, kMax);
    }
    if (frc::SmartDashboard::GetNumber("Shooter Velocity: ", 0) != shooter.kVelocity_) {
        shooter.kVelocity_ = frc::SmartDashboard::GetNumber("Shooter Velocity: ", shooter.kVelocity_);
    }

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

    autonomousCommand = oi.getAutonomousCommand();

    if (autonomousCommand != nullptr) {
        autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
    if (autonomousCommand != nullptr) {
        autonomousCommand->Cancel();
        autonomousCommand = nullptr;
    }

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