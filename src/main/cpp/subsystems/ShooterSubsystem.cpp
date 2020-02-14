#include "subsystems/ShooterSubsystem.h"

using IdleMode = rev::CANSparkMax::IdleMode;
using NeutralMode = ctre::phoenix::motorcontrol::NeutralMode;

ShooterSubsystem::ShooterSubsystem() {
    shooter2.Follow(shooter1, true);

    leftBelt.SetInverted(true);
    bottomBelt.SetInverted(true);

    shooter1.SetSmartCurrentLimit(65);
    shooter2.SetSmartCurrentLimit(65);

    shooter1PID.SetP(kP);
    shooter1PID.SetI(kI);
    shooter1PID.SetD(kD);
    shooter1PID.SetFF(kFF);
    shooter1PID.SetIZone(kIz);
    shooter1PID.SetOutputRange(kMin, kMax);
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

    std::cout << "Speed: " << shooterSpeed << std::endl;
}

void ShooterSubsystem::runShooterVelocity() {
    shooter1PID.SetReference(ShooterVelocity, rev::ControlType::kVelocity);
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
    return shooterEncoder1.GetVelocity();
}

double ShooterSubsystem::getShooter1Current() {
    return shooter1.GetOutputCurrent();
}

double ShooterSubsystem::getShooter1AccumError() {
    shooter1PID.GetIAccum();
}