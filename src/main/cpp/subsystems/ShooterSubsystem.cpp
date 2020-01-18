#include "subsystems/ShooterSubsystem.h"

using IdleMode = rev::CANSparkMax::IdleMode;

ShooterSubsystem::ShooterSubsystem() {
    shooterSlave1.Follow(shooterMaster);
    shooterSlave2.Follow(shooterMaster);
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

    std::cout << "Speed: " << shooterSpeed<< std::endl;
}

void ShooterSubsystem::stopShooter() {
    shooterSpeed = 0;
    shooterMaster.Set(shooterSpeed);
}

void ShooterSubsystem::setCoastMode() {
    shooterMaster.SetIdleMode(IdleMode::kCoast);
    shooterSlave1.SetIdleMode(IdleMode::kCoast);
    shooterSlave2.SetIdleMode(IdleMode::kCoast);
}