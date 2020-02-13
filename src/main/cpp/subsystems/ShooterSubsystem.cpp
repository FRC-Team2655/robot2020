#include "subsystems/ShooterSubsystem.h"

using IdleMode = rev::CANSparkMax::IdleMode;
using NeutralMode = ctre::phoenix::motorcontrol::NeutralMode;

ShooterSubsystem::ShooterSubsystem() {
    shooter1.SetInverted(false);
    shooter2.SetInverted(false);

    leftBelt.SetInverted(true);
    bottomBelt.SetInverted(true);

    shooter1.SetSmartCurrentLimit(65);
    shooter2.SetSmartCurrentLimit(65);
}

void ShooterSubsystem::Periodic() {}

void ShooterSubsystem::runShooterPercentage(double startingSpeed) {
    if (shooterSpeed < startingSpeed) {
        shooterSpeed = startingSpeed;
    }else{
        shooterSpeed += incrementShooterSpeed;
    }

    if (shooterSpeed >= maxShooterSpeed) {
        shooterSpeed = maxShooterSpeed;
    }

    shooter1.Set(shooterSpeed);
    shooter2.Set(-shooterSpeed);

    std::cout << "Speed: " << shooterSpeed<< std::endl;
}

void ShooterSubsystem::runShooterVelocity() {
    shooter1PID.SetReference(kVelocity_, rev::ControlType::kVelocity);
    shooter2PID.SetReference(-kVelocity_, rev::ControlType::kVelocity);
}

void ShooterSubsystem::stopShooter() {
    shooterSpeed = 0.0;
    shooter1.Set(shooterSpeed);
    shooter2.Set(shooterSpeed);
}

void ShooterSubsystem::setCoastMode() {
    shooter1.SetIdleMode(IdleMode::kCoast);
    shooter2.SetIdleMode(IdleMode::kCoast);

    kicker.SetNeutralMode(NeutralMode::Coast);
    leftBelt.SetNeutralMode(NeutralMode::Coast);
    rightBelt.SetNeutralMode(NeutralMode::Coast);
    bottomBelt.SetNeutralMode(NeutralMode::Coast);
}

void ShooterSubsystem::runBelts(double speed) {
    static double firstSwitchTime = 0.0;
    static double secondSwitchTime = 2.0;
    if(frc::Timer::GetFPGATimestamp() < firstSwitchTime)
    {
        leftBelt.Set(speed);
        rightBelt.Set(-speed);
    }
    else if(frc::Timer::GetFPGATimestamp() < secondSwitchTime)
    {
        leftBelt.Set(-speed);
        rightBelt.Set(speed);
    }
    else
    {
        firstSwitchTime = frc::Timer::GetFPGATimestamp() + 1;
        secondSwitchTime = firstSwitchTime + 2;
    }
    
    bottomBelt.Set(speed);
    kicker.Set(speed);
}

void ShooterSubsystem::stopBelts() {
    leftBelt.Set(0);
    rightBelt.Set(0);
    bottomBelt.Set(0);
    kicker.Set(0);
}

double ShooterSubsystem::getRPM() {
    return ((shooterEncoder1.GetVelocity() + -shooterEncoder2.GetVelocity()) / 2.0);
}

double ShooterSubsystem::getShooter1Current() {
    return shooter1.GetOutputCurrent();
}

double ShooterSubsystem::getShooter2Current() {
    return shooter2.GetOutputCurrent();
}

double ShooterSubsystem::getShooter1AccumError() {
    shooter1PID.GetIAccum();
}

double ShooterSubsystem::getShooter2AccumError() {
    shooter2PID.GetIAccum();
}