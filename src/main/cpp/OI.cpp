#include "OI.h"
#include "Robot.h"

using namespace team2655;

const auto kMaxSpeed = 3_mps;
const auto kMaxAcceleration = 3_mps_sq;
const auto kTrackwidth = 0.69_m;
const frc::DifferentialDriveKinematics kDriveKinematics(kTrackwidth);
const auto ka = 0.2 * 1_V * 1_s * 1_s / 1_m;
const auto kv = 1.98 * 1_V * 1_s / 1_m;
const auto ks = 0.22_V;

OI::OI() : odometry(Robot::driveBase.getIMUAngle()) {
  js0 = new frc::Joystick(0);
  
  runButtons();
}

void OI::runButtons() {
  xBtn = new frc2::JoystickButton(js0, 2);
  squareBtn = new frc2::JoystickButton(js0, 1);
  triangleBtn = new frc2::JoystickButton(js0, 4);
  circleBtn = new frc2::JoystickButton(js0, 3);

  xBtn->WhileHeld(new RunShooterCommand(Robot::shooter.kVelocity_));
  squareBtn->WhileHeld(rbCommand);
  triangleBtn->WhileHeld(invertrbCommand);
  circleBtn->WhileHeld(riRollersCommand);
}

frc2::Command* OI::getAutonomousCommand() {
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
    frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
    kDriveKinematics, 10_V
  );

  frc::TrajectoryConfig config(kMaxSpeed, kMaxAcceleration);
  config.SetKinematics(kDriveKinematics);
  config.AddConstraint(autoVoltageConstraint);

  frc::Trajectory m_test = frc::TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/Path1.wpilib.json");

  std::cout << "generated" << std::endl;

  frc2::RamseteCommand ramseteCommand(
    m_test,
    [this]() {return odometry.GetPose();}, 
    frc::RamseteController(kRamseteB, kRamseteZeta),
    frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka), 
    kDriveKinematics,
    [this]() {return Robot::driveBase.getEncoderOutputs();}, 
    frc2::PIDController(kPDriveVel, 0, 0), frc2::PIDController(kPDriveVel, 0, 0), 
    [this](auto left, auto right) {Robot::driveBase.tankDriveVolts(left, right);}, 
    {&Robot::driveBase}
  );

  std::cout << "RamseteCommand over" << std::endl;

  return new frc2::SequentialCommandGroup(
     std::move(ramseteCommand),
     frc2::InstantCommand([this] { Robot::driveBase.tankDriveVolts(0_V, 0_V); }, {}));
}