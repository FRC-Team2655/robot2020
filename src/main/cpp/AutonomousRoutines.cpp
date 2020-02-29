
#include "AutonomousRoutines.h"

frc2::Command* AutonomousRoutines::ShootPreloads(double goalOffsetMeters, double startDelayMs)
{
    /* Create the base sequential command group */
    frc2::SequentialCommandGroup* routine = new frc2::SequentialCommandGroup();

    /* add start delay */
    if(startDelayMs > 0)
    {
        DelayMillisecondsCommand initDelay(startDelayMs);
        routine->AddCommands(initDelay);
    }

    /* Perform first turn */

    /* Perform first drive */

    /* Perform second turn */

    /* Drive remainder of distance to goal while ramping shooter wheel */

    /* Run belts while running shooter wheel */

    /* Back up to the start line */
}

frc2::Command* AutonomousRoutines::TestAuto(double distanceMeters, double turnDegrees)
{
    /* Create the base sequential command group */
    frc2::SequentialCommandGroup* routine = new frc2::SequentialCommandGroup();

    /* Add any sub commands */

    return routine;
}
