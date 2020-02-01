#include "subsystems/ShooterSubsystem.h"

using IdleMode = rev::CANSparkMax::IdleMode;

ShooterSubsystem::ShooterSubsystem() {
    shooterSlave1.Follow(shooterMaster);

    beltBottom.Follow(beltForward);
    kicker.Follow(beltForward);
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

    shooterMaster.Set(shooterSpeed);
}

void ShooterSubsystem::runBelts(double speed) {
    beltForward.Set(speed);
    beltBackward.Set(-speed);
}

void ShooterSubsystem::stopBelts() {
    beltForward.Set(0);
    beltBackward.Set(0);
}

void ShooterSubsystem::stopShooter() {
    shooterSpeed = 0;
    shooterMaster.Set(shooterSpeed);
}

void ShooterSubsystem::setCoastMode() {
    shooterMaster.SetIdleMode(IdleMode::kCoast);
    shooterSlave1.SetIdleMode(IdleMode::kCoast);

    beltForward.SetIdleMode(IdleMode::kCoast);
    beltBackward.SetIdleMode(IdleMode::kCoast);
    beltBottom.SetIdleMode(IdleMode::kCoast);
}