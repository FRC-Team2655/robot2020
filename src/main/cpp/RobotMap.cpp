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

const int RMaster = 6;
const int RSlave1 = 5;
const int RSlave2 = 4;

const double MaxVelocity = 5500;    // This is capped at the slowest velocity on ANY robot to ensure that paths work the same
const double LMaxVelocity = 5700.0;
const double RMaxVelocity = 5800.0;

const double DriveRampRate = 0.23;  // Minimum time (sec) to go from 0 to full

const double kRamseteB = 2;
const double kRamseteZeta = 0.7;

const double kPDriveVel = 8.5;

const int LEncA = 5;
const int LEncB = 6;
const int REncA = 1;
const int REncB = 2;

const int Shooter1ID = 7;
const int Shooter2ID = 8;

const double incrementShooterSpeed = 0.05;
const double maxShooterSpeed = 0.9;

const int KickerID = 13;

const int BeltLeft = 10;
const int BeltRight = 11;
const int BeltBottom = 12;

const int RollerShooters = 14;

const int IntakePWM = 4;
const int IntakeArm = 9;