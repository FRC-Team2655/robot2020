#include "OI.h"
#include "Robot.h"

using namespace team2655;

OI::OI() {
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
  optionsBtn = new frc2::JoystickButton(js0, 10);

  l2Btn->WhileHeld(rsVelocityCommand);
  
  r2Btn->WhileHeld(rbCommand, false);
  triangleBtn->WhileHeld(invertrbCommand);
  shareBtn->WhenPressed(updateIntakeOffsetUp);
  optionsBtn->WhenPressed(updateIntakeOffsetDown);

  r1Btn->WhileHeld(riRollersCommand);
  circleBtn->WhenPressed(frc2::SequentialCommandGroup(MoveIntakeOutArmCommand(-0.3), RunBeltsBackgroundCommand(0.5)), true);
  squareBtn->WhenPressed(frc2::SequentialCommandGroup(MoveIntakeInArmCommand(0), IntakeArmLockPIDCommand()), true);
}