/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <rev/CANSparkMax.h>
#include "RobotMap.h"
#include "commands/DriveJoystickCommand.h"

#include <frc/PIDController.h>

using MotorType = rev::CANSparkMax::MotorType;

class DriveBaseSubsystem : public frc::Subsystem {
public:
  DriveBaseSubsystem();

  void InitDefaultCommand() override;

  /**
   * Drive in percentage mode (arcade drive style)
   * @param speed speed to drive (power)
   * @param rotation Percent rotation
   */
  void drivePercentage(double speed, double rotation);

  /**
   * Drive in velocity mode (arcade drive style)
   * @param speed Speed to drive (power)
   * @param rotation Percent rotation
   */ 
  void driveVelocity(double speed, double rotation);

  /**
   * Drive in velocity mode not using arcade drive.
   * @param lVel The velocity for the left side.
   * @param rVel The velocity for the right side.
   */
  void driveTankVelocity(double lVel, double rVel);

  /**
   * Drive in percentage mode not using arcade drive.
   * @param leftPercentage The percentage to drive the left side.
   * @param rightPercentage The percentage to drive the right side.
   */ 
  void driveTankPercentage(double leftPercentage, double rightPercentage);

  // Setting coast mode for the motors.
  void setCoastMode();

  // Setting brake mode for the motors.
  void setBrakeMode();

  std::array<double, 2> arcadeDrive(double xSpeed, double zRotation);

 private:
  rev::CANSparkMax leftMaster {LMaster, MotorType::kBrushless};
  rev::CANSparkMax leftSlave {LSlave, MotorType::kBrushless};
  rev::CANSparkMax leftSlave2 {LSlave2, MotorType::kBrushless};
  rev::CANSparkMax rightMaster {RMaster, MotorType::kBrushless};
  rev::CANSparkMax rightSlave {RSlave, MotorType::kBrushless};
  rev::CANSparkMax rightSlave2 {RSlave2, MotorType::kBrushless};

  rev::CANPIDController leftPID = leftMaster.GetPIDController();
  rev::CANPIDController rightPID = rightMaster.GetPIDController();
};
