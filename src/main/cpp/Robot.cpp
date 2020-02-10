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

void Robot::RobotInit() {
    frc::SmartDashboard::PutBoolean("Reset Encoders", false);
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

    if (frc::SmartDashboard::GetBoolean("Reset Encoders", false) == true) {
        driveBase.resetEncoders();
        frc::SmartDashboard::PutBoolean("Reset Encoders", false);
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