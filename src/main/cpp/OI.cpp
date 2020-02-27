#include "OI.h"
#include "Robot.h"

using namespace team2655;

const auto kMaxSpeed = 3_mps;
const auto kMaxAcceleration = 3_mps_sq;
const auto kTrackwidth = 0.226_m;
const frc::DifferentialDriveKinematics kDriveKinematics(kTrackwidth);
const auto ka = 0.2 * 1_V * 1_s * 1_s / 1_m;
const auto kv = 1.98 * 1_V * 1_s / 1_m;
const auto ks = 0.22_V;
const double kRamseteB = 2;
const double kRamseteZeta = 0.7;
const double kPDriveVel = 8.5;

OI::OI() : odometry(Robot::driveBase.getAutoIMUAngle()) {
  js0 = new frc::Joystick(0);
  
  runButtons();
}

void OI::runButtons() {
  xBtn = new frc2::JoystickButton(js0, 2);
  squareBtn = new frc2::JoystickButton(js0, 1);
  triangleBtn = new frc2::JoystickButton(js0, 4);
  circleBtn = new frc2::JoystickButton(js0, 3);
  l2Btn = new frc2::JoystickButton(js0, 7);
  r2Btn = new frc2::JoystickButton(js0, 8);
  r1Btn = new frc2::JoystickButton(js0, 6);
  shareBtn = new frc2::JoystickButton(js0, 9);

  l2Btn->WhileHeld(rsVelocityCommand);
  r2Btn->WhileHeld(rbCommand, false);
  triangleBtn->WhileHeld(invertrbCommand);

  r1Btn->WhileHeld(riRollersCommand);
  circleBtn->WhenPressed(frc2::SequentialCommandGroup(MoveIntakeOutArmCommand(-0.3), RunBeltsBackgroundCommand(0.5)), true);
  squareBtn->WhenPressed(miInCommand);
}

frc2::Command* OI::getAutonomousCommand() {
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
    frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
    kDriveKinematics, 10_V
  );

  frc::TrajectoryConfig config(kMaxSpeed, kMaxAcceleration);
  config.SetKinematics(kDriveKinematics);
  config.AddConstraint(autoVoltageConstraint);

  std::cout << "config" << std::endl;

  //frc::Trajectory m_test = frc::TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/Path1.wpilib.json");
  auto m_test = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      {},
      frc::Pose2d(1_m, 0_m, frc::Rotation2d(0_deg)),
      config);

    std::cout << "Trajectory Generated!" << std::endl;

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

  return new frc2::SequentialCommandGroup(
     std::move(ramseteCommand),
     frc2::InstantCommand([this] { Robot::driveBase.tankDriveVolts(0_V, 0_V); }, {}));
}