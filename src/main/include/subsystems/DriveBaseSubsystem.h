/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "RobotMap.h"
#include "commands/DriveJoystickCommand.h"

#include <rev/CANSparkMax.h>

using MotorType = rev::CANSparkMax::MotorType;

class DriveBaseSubsystem : public frc2::SubsystemBase {
 public:
  DriveBaseSubsystem();

  void Periodic();

  void drivePercentage(double speed, double rotation);

  void driveTankPercentage(double lVel, double rVel);

  std::array<double, 2> arcadeDrive(double xSpeed, double zRotation);

  void driveTankVelocity(double lVel, double rVel);

  void driveVelocity(double speed, double rotation);

  void setCoastMode();
  
  void setBrakeMode();

 private:
  rev::CANSparkMax leftMaster {LMaster, MotorType::kBrushless};
  //rev::CANSparkMax leftSlave1 {LSlave1, MotorType::kBrushless};
  //rev::CANSparkMax leftSlave2 {LSlave2, MotorType::kBrushless};
  rev::CANSparkMax rightMaster {RMaster, MotorType::kBrushless};
  //rev::CANSparkMax rightSlave1 {RSlave1, MotorType::kBrushless};
  //rev::CANSparkMax rightSlave2 {RSlave2, MotorType::kBrushless};

  rev::CANPIDController leftPID = leftMaster.GetPIDController();
  rev::CANPIDController rightPID = rightMaster.GetPIDController();

  DriveJoystickCommand driveJoystick;
};
