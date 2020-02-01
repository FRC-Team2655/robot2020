/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <units/units.h>
#include <wpi/math>

const extern int LMaster;
const extern int LSlave1;
const extern int LSlave2;

const extern int RMaster;
const extern int RSlave1;
const extern int RSlave2;

const extern int ShooterMaster;
const extern int ShooterSlave1;
const extern int ShooterSlave2;

const extern int rollersIntake;
const extern int intakeMotor;

const extern int KickerID;

const extern int ForwardBelt;
const extern int BackwardBelt;
const extern int BottomBelt;

const extern double incrementShooterSpeed;
const extern double maxShooterSpeed;

const extern double incrementIntakeSpeed;
const extern double maxIntakeSpeed;

const extern double MaxVelocity;
const extern double LMaxVelocity;
const extern double RMaxVelocity;

const extern double DriveRampRate;

const extern frc::DifferentialDriveKinematics kDriveKinematics;

const extern double kRamseteB;
const extern double kRamseteZeta;

const extern double kPDriveVel;