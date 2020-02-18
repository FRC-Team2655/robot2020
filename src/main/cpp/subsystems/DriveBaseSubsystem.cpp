/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveBaseSubsystem.h"
#include "Robot.h"
#include <math.h>

using IdleMode = rev::CANSparkMax::IdleMode;

DriveBaseSubsystem::DriveBaseSubsystem() {
  leftEncoder.SetDistancePerPulse(0);
  rightEncoder.SetDistancePerPulse(0);

  leftSlave1.Follow(leftMaster);
  leftSlave2.Follow(leftMaster);
  rightSlave1.Follow(rightMaster);
  rightSlave2.Follow(rightMaster);

  leftMaster.SetClosedLoopRampRate(DriveRampRate);
  rightMaster.SetClosedLoopRampRate(DriveRampRate);

  // Current limiting
  const int stallLimit = 38, freeLimit = 60, limitRPM = 2000;
  leftMaster.SetSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
  leftSlave1.SetSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
  leftSlave2.SetSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
  rightMaster.SetSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
  rightSlave1.SetSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
  rightSlave2.SetSmartCurrentLimit(stallLimit, freeLimit, limitRPM);


  leftMaster.BurnFlash();
  rightMaster.BurnFlash();

  SetDefaultCommand(driveJoystick);

  rightMaster.SetInverted(false);
  leftMaster.SetInverted(true);
}

// This method will be called once per scheduler run
void DriveBaseSubsystem::Periodic() {
}

void DriveBaseSubsystem::drivePercentage(double speed, double rotation){
	std::array<double, 2> speeds = arcadeDrive(speed, rotation);
	driveTankPercentage(speeds[0], speeds[1]);
}

void DriveBaseSubsystem::driveTankPercentage(double leftPercentage, double rightPercentage) {
	leftMaster.Set(leftPercentage);
	leftSlave1.Set(leftPercentage);
	leftSlave2.Set(leftPercentage);

	rightMaster.Set(rightPercentage);
	rightSlave1.Set(rightPercentage);
	rightSlave2.Set(rightPercentage);
}

std::array<double, 2> DriveBaseSubsystem::arcadeDrive(double xSpeed, double zRotation) {

	double leftMotorOutput;
	double rightMotorOutput;

	// Prevent -0 from breaking the arcade drive...
	xSpeed += 0.0;
	zRotation += 0.0;

	double maxInput = std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);

	if (xSpeed >= 0.0) {
		// First quadrant, else second quadrant
		if (zRotation >= 0.0) {
			leftMotorOutput = maxInput;
			rightMotorOutput = xSpeed - zRotation;
		} else {
			leftMotorOutput = xSpeed + zRotation;
			rightMotorOutput = maxInput;
		}
	} else {
		// Third quadrant, else fourth quadrant
		if (zRotation >= 0.0) {
			leftMotorOutput = xSpeed + zRotation;
			rightMotorOutput = maxInput;
		} else {
			leftMotorOutput = maxInput;
			rightMotorOutput = xSpeed - zRotation;
		}
	}

	return { leftMotorOutput, rightMotorOutput };
}

void DriveBaseSubsystem::driveVelocity(double speed, double rotation) {
	std::array<double, 2> speeds = arcadeDrive(speed, rotation);
	speeds[0] *= MaxVelocity;
	speeds[1] *= MaxVelocity;
	driveTankVelocity(speeds[0], speeds[1]);
}

void DriveBaseSubsystem::driveTankVelocity(double lVel, double rVel) {
	if (lVel == 0) {
		// If target velocity is 0 do not use PID to get to 0 just cut power (0%)
		leftMaster.Set(0);
	}else {
		// Drive the left side in velocity closed loop mode (set pid reference = setpoint for PID)
		leftMaster.Set(leftPID.Calculate(leftEncoder.GetRate(), lVel));
	}

	if (rVel == 0) {
		rightMaster.Set(0);
	}
	
	else {
		rightMaster.Set(rightPID.Calculate(rightEncoder.GetRate(), -rVel));
	}
}

void DriveBaseSubsystem::setBrakeMode() {
	leftMaster.SetIdleMode(IdleMode::kBrake);
	leftSlave1.SetIdleMode(IdleMode::kBrake);
	leftSlave2.SetIdleMode(IdleMode::kBrake);

	rightMaster.SetIdleMode(IdleMode::kBrake);
	rightSlave1.SetIdleMode(IdleMode::kBrake);
	rightSlave2.SetIdleMode(IdleMode::kBrake);
}

void DriveBaseSubsystem::setCoastMode() {
	leftMaster.SetIdleMode(IdleMode::kCoast);
	leftSlave1.SetIdleMode(IdleMode::kCoast);
	leftSlave2.SetIdleMode(IdleMode::kCoast);

	rightMaster.SetIdleMode(IdleMode::kCoast);
	rightSlave1.SetIdleMode(IdleMode::kCoast);
	rightSlave2.SetIdleMode(IdleMode::kCoast);
}

double DriveBaseSubsystem::getLeftEncoderOutput() {
	return (leftEncoder.GetRaw() / 8192.0);
}

double DriveBaseSubsystem::getRightEncoderOutput() {
	return (rightEncoder.GetRaw() / 8192.0);
}

frc::DifferentialDriveWheelSpeeds DriveBaseSubsystem::getEncoderOutputs() {
	return {units::meters_per_second_t(leftEncoder.GetRate()),
			units::meters_per_second_t(rightEncoder.GetRate())};
}

void DriveBaseSubsystem::tankDriveVolts(units::volt_t left, units::volt_t right) {
	leftMaster.SetVoltage(left);
	leftSlave1.SetVoltage(left);
	leftSlave2.SetVoltage(left);
	rightMaster.SetVoltage(-right);
	rightSlave1.SetVoltage(-right);
	rightSlave2.SetVoltage(-right);
}

void DriveBaseSubsystem::resetEncoders() {
	leftEncoder.Reset();
	rightEncoder.Reset();
}

frc::Rotation2d DriveBaseSubsystem::getIMUAngle() {
	return frc::Rotation2d((units::radian_t)(imu.GetAngle() * 3.141592 / 180.0));
}

double DriveBaseSubsystem::leftCurrent() {
	return leftMaster.GetOutputCurrent();
}

double DriveBaseSubsystem::rightCurrent() {
	return rightMaster.GetOutputCurrent();
}