#pragma once

#include <vector>
#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include "RobotMap.h"

/* Auto specific commands */
#include "commands/DriveDistanceCommand.h"
#include "commands/RotateDegreesCommand.h"
#include "commands/DelayMillisecondsCommand.h"
#include "commands/RotateToGyroAngleCommand.h"

/* Driver commands being used for auto */
#include "commands/MoveIntakeInArmCommand.h"
#include "commands/MoveIntakeOutArmCommand.h"
#include "commands/RunBeltsCommand.h"
#include "commands/RunShooterVelocityCommand.h"
#include "commands/RunIntakeRollersCommand.h"

/* Class to hold all defined auto routines */
class AutonomousRoutines
{
public:
    double driveDistance_P_gyro;
    double driveDistance_P_encoder;
    double rotate_P_gyro;
    frc2::Command* ShootPreloads(double goalOffsetMeters, double startDelayMs);
    frc2::Command* TestAuto(double distanceMeters, double turnDegrees);
    frc2::Command* PickupFromTrechAndShoot(double gyroStartAngle);
private:
};