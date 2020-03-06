#include "OI.h"
#include "Robot.h"

using namespace team2655;

OI::OI() {
  js0 = new frc::Joystick(0);
  js1 = new frc::Joystick(1);
  
  runButtons();
}

void OI::runButtons() {
  xBtn = new frc2::JoystickButton(js0, 2);
  xBtn1 = new frc2::JoystickButton(js1, 2);
  squareBtn = new frc2::JoystickButton(js0, 1);
  triangleBtn = new frc2::JoystickButton(js0, 4);
  triangleBtn1 = new frc2::JoystickButton(js1, 4);
  circleBtn = new frc2::JoystickButton(js0, 3);
  l2Btn = new frc2::JoystickButton(js0, 7);
  l2Btn1 = new frc2::JoystickButton(js1, 7);
  r2Btn = new frc2::JoystickButton(js0, 8);
  r2Btn1 = new frc2::JoystickButton(js1, 8);
  r1Btn = new frc2::JoystickButton(js0, 6);
  r1Btn1 = new frc2::JoystickButton(js1, 6);
  shareBtn = new frc2::JoystickButton(js0, 9);
  optionsBtn = new frc2::JoystickButton(js0, 10);

  l2Btn->WhileHeld(rsVelocityCommand);
  l2Btn1->WhileHeld(invertrbCommand);
  
  r2Btn->WhileHeld(rbCommand, false);
  r2Btn1->WhileHeld(rsVelocityCommand);
  triangleBtn->WhileHeld(invertrbCommand);
  shareBtn->WhenPressed(updateIntakeOffsetUp);
  xBtn1->WhenPressed(updateIntakeOffsetUp);
  triangleBtn1->WhenPressed(updateIntakeOffsetDown);
  optionsBtn->WhenPressed(updateIntakeOffsetDown);

  r1Btn->WhileHeld(riRollersCommand);
  r1Btn1->WhileHeld(riRollersCommand);
  circleBtn->WhenPressed(frc2::SequentialCommandGroup(MoveIntakeOutArmCommand(-0.3), RunBeltsBackgroundCommand(0.5)), true);
  squareBtn->WhenPressed(frc2::SequentialCommandGroup(MoveIntakeInArmCommand(0), IntakeArmLockPIDCommand()), true);
}