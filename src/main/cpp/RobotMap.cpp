/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "RobotMap.h"

const int LMaster = 1;
const int LSlave1 = 2;
const int LSlave2 = 3;

const int RMaster = 4;
const int RSlave1 = 5;
const int RSlave2 = 6;

const int rollersIntake = 13;
const int intakeMotor = 14;

const int ShooterMaster = 11;
const int ShooterSlave1 = 12;

const int KickerID = 10;

const int ForwardBelt = 7;
const int BackwardBelt = 8;
const int BottomBelt = 9;

const double incrementShooterSpeed = 0.05;
const double maxShooterSpeed = 0.75;

const double incrementIntakeSpeed = 0.05;
const double maxIntakeSpeed = 0.50;

const double MaxVelocity = 5500;    // This is capped at the slowest velocity on ANY robot to ensure that paths work the same
const double LMaxVelocity = 5700.0;
const double RMaxVelocity = 5800.0;

const double DriveRampRate = 0.23;  // Minimum time (sec) to go from 0 to full

const double kRamseteB = 2;
const double kRamseteZeta = 0.7;

const double kPDriveVel = 8.5;