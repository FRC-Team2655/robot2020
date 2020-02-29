/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RotateDegreesCommand.h"
#include "Robot.h"

RotateDegreesCommand::RotateDegreesCommand(double degrees) : degrees(degrees) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RotateDegreesCommand::Initialize() {
  /* First mod by 360 to get to range -360 - 360 */
  degrees = std::fmod(degrees, 360);

  /* Correct for out of range degree values by add/subtracting full rotation */
  if(degrees < -180)
    degrees = degrees + 360;
  else if(degrees > 180)
    degrees = degrees - 360;

  /* degrees is now in range [-180 - 180]. Get the gyro start angle */
  startAngle = Robot::driveBase.getIMUAngle();
  currentAngle = startAngle;

  /* Find the target angle */
  targetAngle = startAngle + degrees;

  /* Find rotate direction */
  if(degrees < 0)
    turnRight = true;
  else
    turnRight = false;

  /* Init the current RPM value to min */
  currentSpeed = minMotorRPM;
}

// Called repeatedly when this Command is scheduled to run
void RotateDegreesCommand::Execute() {
  double remainingAngle;
  double rampDownCompensation;

  /* Get the IMU angle */
  currentAngle = Robot::driveBase.getIMUAngle();
  
  /* Calculate remaining angle */
  remainingAngle = std::abs(targetAngle - currentAngle);

  if(remainingAngle < rampdownStartAngle)
  {
    /* Need to start slowing down */
    rampDownCompensation = (rampdownStartAngle - remainingAngle);
    rampDownCompensation *= P_gyro;
    currentSpeed -= rampDownCompensation;
  }
  else
  {
    if(currentSpeed < maxMotorRPM)
    {
      /* Ramp up motor speed */
      currentSpeed += rampUpIncrement;
    }
    else
    {
      /* Cap at max */
      currentSpeed = maxMotorRPM;
    }
  }

  /* Apply speed clamping */
  if(currentSpeed < minMotorRPM)
    currentSpeed = minMotorRPM;
  if(currentSpeed > maxMotorRPM)
    currentSpeed = maxMotorRPM;

  /* Apply the current speed value to motors */
  if(turnRight)
      Robot::driveBase.driveTankVelocity(currentSpeed, -currentSpeed);
  else
    Robot::driveBase.driveTankVelocity(-currentSpeed, currentSpeed);
}

// Called once the command ends or is interrupted.
void RotateDegreesCommand::End(bool interrupted) {
  Robot::driveBase.driveTankVelocity(0, 0);
}

// Returns true when the command should end.
bool RotateDegreesCommand::IsFinished() {
  if(turnRight) /* negative angle */
    return (currentAngle <= targetAngle);
  else /* Postive angle */
    return (currentAngle >= targetAngle);
}
