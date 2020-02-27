/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveDistanceCommand.h"
#include "Robot.h"

DriveDistanceCommand::DriveDistanceCommand(double distance) : distance(distance) {
  AddRequirements(&Robot::driveBase);
}

// Called when the command is initially scheduled.
void DriveDistanceCommand::Initialize() {
  Robot::driveBase.driveTankPercentage(0, 0);
  Robot::driveBase.resetEncoders();
  currentDistance = 0;
  currentSpeed = 0;
  gyroStartAngle = Robot::driveBase.getIMUAngle();
  gyroAngle = gyroStartAngle;
  gyroSpeedCompensation = 0;
}

// Called repeatedly when this Command is scheduled to run
void DriveDistanceCommand::Execute() {
  double remainingDistance;
  double gyroError;
  double leftSpeed, rightSpeed;
  //get travelled distance
  currentDistance = GetCurrentDistance();
  //calculate remaining distance
  remainingDistance = distance - currentDistance;

  if(remainingDistance < rampDownDistance)
  {
    /* Slowing down. Want to go from maxspeed to 0 over 1 meter */
    currentSpeed -= (P_encoders * (rampDownDistance - remainingDistance));
    if(currentSpeed < minSpeed)
      currentSpeed = minSpeed;
  }
  else
  {
    /* Ramping up speed */
    if(currentSpeed < minSpeed)
    {
      currentSpeed = minSpeed;
    }
    else if(currentSpeed < maxSpeed)
    {
      currentSpeed += speedStep;
    }
    else
    {
      currentSpeed = maxSpeed;
    }
  }
  
  /* handle gyro compensation */

  /* Resample the gyro */
  gyroAngle = Robot::driveBase.getIMUAngle();
  /* Calculate the error (in degrees) */
  gyroError = gyroAngle - gyroStartAngle;
  
  /* Accumulate (feedback error * gyro P value) into gyro speed compensation value */
  gyroSpeedCompensation = (P_gyro * gyroError);

  /* Apply gyro compensation */
  leftSpeed = currentSpeed + gyroSpeedCompensation;
  rightSpeed = currentSpeed - gyroSpeedCompensation;

  /* Check speed capping */
  if(leftSpeed > maxSpeed)
  {
    rightSpeed *= (maxSpeed / leftSpeed);
    leftSpeed = maxSpeed;
  }
  if(rightSpeed > maxSpeed)
  {
    leftSpeed *= (maxSpeed / rightSpeed);
    rightSpeed = maxSpeed;
  }

  /* Apply to drive base */
  Robot::driveBase.driveTankPercentage(leftSpeed, rightSpeed);

  std::cout << "Left: " << leftSpeed << ", " << "Right: " << rightSpeed << ", " << "Gyro Error: " << gyroError << std::endl;
}

// Called once the command ends or is interrupted.
void DriveDistanceCommand::End(bool interrupted) {
  Robot::driveBase.driveTankPercentage(0, 0);
}

// Returns true when the command should end.
bool DriveDistanceCommand::IsFinished() {
  return (currentDistance >= distance);
 }

 double DriveDistanceCommand::GetCurrentDistance()
 {
   double averageRotations;
   averageRotations = Robot::driveBase.getRightEncoderRotations()+ Robot::driveBase.getLeftEncoderRotations();
   averageRotations *= 0.5;
   //scale to meters. Wheels are 6" (15cm) diameter
   return averageRotations * 3.141592 * 0.1524;
 }
