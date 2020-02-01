#include "Autonomous.h"
#include "Robot.h"

using namespace team2655;

const auto kMaxSpeed = 3_mps;
const auto kMaxAcceleration = 3_mps_sq;
const auto kTrackwidth = 0.69_m;
const frc::DifferentialDriveKinematics kDriveKinematics(kTrackwidth);
const auto ka = 0.2 * 1_V * 1_s * 1_s / 1_m;
const auto kv = 1.98 * 1_V * 1_s / 1_m;
const auto ks = 0.22_V;

Autonomous::Autonomous() : odometry(Robot::driveBase.getIMUAngle()){

}

frc2::Command* Autonomous::getAutonomousCommand() {
  std::cout << "Started" << std::endl;

  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
    frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
    kDriveKinematics, 10_V);

    frc::TrajectoryConfig config(kMaxSpeed, kMaxAcceleration);
    config.SetKinematics(kDriveKinematics);
    config.AddConstraint(autoVoltageConstraint);

    frc::Trajectory trajectoryTest = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)), config
  );

    std::cout << "Trajectory generated" << std::endl;

  frc2::RamseteCommand ramseteCommand(
    trajectoryTest, 
    [this]() {return odometry.GetPose();}, 
    frc::RamseteController(kRamseteB, kRamseteZeta),
    frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka), 
    kDriveKinematics,
    [this]() {return Robot::driveBase.getEncoderOutputs();}, 
    frc2::PIDController(kPDriveVel, 0, 0), frc2::PIDController(kPDriveVel, 0, 0), 
    [this](auto left, auto right) {Robot::driveBase.tankDriveVolts(left, right);}, 
    {&Robot::driveBase}
  );

  std::cout << "Run command" << std::endl;

  return new frc2::SequentialCommandGroup(
     std::move(ramseteCommand),
     frc2::InstantCommand([this] { Robot::driveBase.tankDriveVolts(0_V, 0_V); }, {}));
}