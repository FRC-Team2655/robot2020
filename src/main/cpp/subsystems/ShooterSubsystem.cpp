#include "subsystems/ShooterSubsystem.h"

using IdleMode = rev::CANSparkMax::IdleMode;
using NeutralMode = ctre::phoenix::motorcontrol::NeutralMode;

ShooterSubsystem::ShooterSubsystem() {
    shooter1.SetInverted(false);
    shooter2.SetInverted(false);
}

void ShooterSubsystem::Periodic() {}

void ShooterSubsystem::runShooter(double startingSpeed) {
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
    leftBelt.Set(speed);
    bottomBelt.Set(speed);
    rightBelt.Set(-0.5 * speed);
}

void ShooterSubsystem::stopBelts() {
    leftBelt.Set(0);
    rightBelt.Set(0);
    bottomBelt.Set(0);
}