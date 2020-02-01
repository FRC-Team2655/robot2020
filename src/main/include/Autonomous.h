#pragma once

#include "RobotMap.h"

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>

class Autonomous {
public:
  Autonomous();

  frc::DifferentialDriveOdometry odometry;
  frc2::Command* getAutonomousCommand();
};