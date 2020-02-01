#include "OI.h"
#include "Robot.h"

using namespace team2655;

OI::OI(){
  js0 = new frc::Joystick(0);
  
  runButtons();
}

void OI::runButtons() {
  xBtn = new frc2::JoystickButton(js0, 2);
  squareBtn = new frc2::JoystickButton(js0, 1);

  xBtn->WhileHeld(rsCommand);
  squareBtn->WhileHeld(riCommand);
}