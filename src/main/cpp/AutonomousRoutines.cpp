
#include "AutonomousRoutines.h"

frc2::Command* AutonomousRoutines::ShootPreloads(double goalOffsetMeters, double startDelayMs)
{
    /* Create the base sequential command group */
    frc2::SequentialCommandGroup* routine = new frc2::SequentialCommandGroup();

    /* straight distance to drive (meters) */
    double straightDriveDist = 2.36;

    /* add start delay */
    if(startDelayMs > 0)
    {
        routine->AddCommands(DelayMillisecondsCommand(startDelayMs));
    }

    /* If we need to do some turns to line up */
    if(goalOffsetMeters != 0)
    {
        /* Perform first turn */

        /* Perform first drive */

        /* Perform second turn */
    }

    /* Drive remainder of distance to goal while ramping shooter wheel */
    routine->AddCommands(frc2::ParallelRaceGroup(DriveDistanceCommand(straightDriveDist), RunShooterVelocityCommand()));

    /* Run belts while running shooter wheel */
    routine->AddCommands(frc2::ParallelRaceGroup(RunShooterVelocityCommand(), RunBeltsCommand(beltsSpeed), DelayMillisecondsCommand(5000)));

    /* Back up to the start line */
    routine->AddCommands(DriveDistanceCommand(-2.95));

    return routine;
}

frc2::Command* AutonomousRoutines::TestAuto(double distanceMeters, double turnDegrees)
{
    /* Create the base sequential command group */
    frc2::SequentialCommandGroup* routine = new frc2::SequentialCommandGroup();

    /* Add any sub commands */
    DriveDistanceCommand driveDist(distanceMeters);
    driveDist.P_encoders = driveDistance_P_encoder;
    driveDist.P_gyro = driveDistance_P_gyro;
    routine->AddCommands(driveDist);

    RotateDegreesCommand rot(turnDegrees);
    rot.P_gyro = rotate_P_gyro;
    routine->AddCommands(rot);

    return routine;
}