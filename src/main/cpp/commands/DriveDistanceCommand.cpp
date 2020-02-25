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
}

// Called repeatedly when this Command is scheduled to run
void DriveDistanceCommand::Execute() {
  double remainingDistance;
  //get travelled distance
  currentDistance = GetCurrentDistance();
  //calculate remaining distance
  remainingDistance = distance - currentDistance;

  if(remainingDistance < 1.0)
  {
    /* Slowing down. Want to go from maxspeed to 0 over 1 meter */
    currentSpeed -= (P * remainingDistance);
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
  
  std::cout << currentDistance << std::endl;

  //todo: Apply gyro compensation
  Robot::driveBase.driveTankPercentage(currentSpeed, currentSpeed);
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
