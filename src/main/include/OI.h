#pragma once

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Button.h>

#include "RobotMap.h"
#include "team2655/joystick.hpp"

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
#include <frc/trajectory/TrajectoryUtil.h>

#include "commands/RunShooterPercentageCommand.h"
#include "commands/RunShooterVelocityCommand.h"
#include "subsystems/ShooterSubsystem.h"
#include "commands/RunBeltsCommand.h"
#include "commands/RunIntakeRollersCommand.h"
#include "commands/MoveIntakeInArmCommand.h"
#include "commands/MoveIntakeOutArmCommand.h"
#include "commands/RunBeltsBackgroundCommand.h"
#include <frc2/command/SequentialCommandGroup.h>
#include "commands/RunBeltsInvertedCommand.h"
#include "commands/MoveArmManualCommand.h"

#include "commands/RunBottomBeltCommand.h"
#include "commands/RunKickerBeltCommand.h"
#include "commands/RunSideBeltsCommand.h"

#include <frc/Joystick.h>

using namespace team2655;

class OI {
public:
  OI();
  frc::Joystick *js0;
  frc2::JoystickButton *xBtn;
  frc2::JoystickButton *squareBtn;  
  frc2::JoystickButton *triangleBtn;
  frc2::JoystickButton *circleBtn;
  frc2::JoystickButton *l2Btn;
  frc2::JoystickButton *r2Btn;
  frc2::JoystickButton *r1Btn;
  frc2::JoystickButton *shareBtn;
 
  RunShooterVelocityCommand rsVelocityCommand {};
  RunBeltsCommand rbCommand {beltsSpeed};
  RunBeltsInvertedCommand invertrbCommand {0.8 * beltsSpeed};
  RunIntakeRollersCommand riRollersCommand {rollersSpeed};
  MoveIntakeInArmCommand miInCommand {intakeInPosition};
  MoveIntakeOutArmCommand  miOutCommand {intakeOutPosition};
  MoveArmManualCommand maManCommand {};

  RunKickerBeltCommand rkBeltCommand {};
  RunSideBeltsCommand rsBeltsCommand {};
  RunBottomBeltCommand rbBeltsCommand {};

  // Configurations for the joystick deadband and cubic function.
  jshelper::AxisConfig driveAxisConfig = jshelper::createAxisConfig(.1, 0, 0.5);
  jshelper::AxisConfig rotateAxisConfig = jshelper::createAxisConfig(0.1);

  frc::DifferentialDriveOdometry odometry;
  
  void runButtons();
  frc2::Command* getAutonomousCommand();
};