
#include "AutonomousRoutines.h"

frc2::Command* AutonomousRoutines::PickupFromTrechAndShoot(double gyroStartAngle)
{
    /* This auto routine is designed to pick up two balls from the trench, drive to the goal, and shoot all 5 balls.
    The robot start position is on the start line, centered with the balls. The robot must be facing the balls,
    and the back of the robot must be in line with the start line (back of chassis, not back of bumper) */

    /* Create the base sequential command group */
    frc2::SequentialCommandGroup* routine = new frc2::SequentialCommandGroup();

    /* Put the intake out */
    routine->AddCommands(MoveIntakeOutArmCommand(intakeOutPosition));

    /* Run rollers and drive distance (~6'6" + 3' + 3' + margin - robot length (27"))
    Distance: 78 + 36 + 36 + 12 - 27 = 135 inches (3.43m) */
    routine->AddCommands(frc2::ParallelRaceGroup(RunIntakeRollersCommand(rollersSpeed), DriveDistanceCommand(3.43, 1500)));

    /* Bring intake in */
    routine->AddCommands(MoveIntakeInArmCommand(intakeInPosition));

    /* Rotate to point where robot *should* end up 1.5 meters from goal */
    routine->AddCommands(RotateDegreesCommand(-164.6));

    /* Drive to 1.5m from goal */
    routine->AddCommands(DriveDistanceCommand(5.16));
    
    /* Re-orient based on gyro start angle to align to goal (180 - start angle) */

    /* Drive 1.7m while revving shooter (want to bump wall) */
    routine->AddCommands(frc2::ParallelRaceGroup(DriveDistanceCommand(1.7), RunShooterVelocityCommand()));

    /* Run belts while running shooter wheel */
    routine->AddCommands(frc2::ParallelRaceGroup(RunShooterVelocityCommand(), RunBeltsCommand(beltsSpeed), DelayMillisecondsCommand(5000)));

    /* Back up to the start line */
    routine->AddCommands(DriveDistanceCommand(-2.95));

    return routine;
}

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