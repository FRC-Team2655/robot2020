#include "RobotMap.h"

// SPARK MAX IDs
const int LMaster = 1;
const int LSlave = 2;
const int LSlave2 = 6;
const int RMaster = 3;
const int RSlave = 4;
const int RSlave2 = 7;

const double MaxVelocity = 5500;    // This is capped at the slowest velocity on ANY robot to ensure that paths work the same
const double LMaxVelocity = 5700.0;
const double RMaxVelocity = 5800.0;

const double DriveRampRate = 0.23;  // Minimum time (sec) to go from 0 to full