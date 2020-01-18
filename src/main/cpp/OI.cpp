#include "OI.h"
#include "Robot.h"

using namespace team2655;

OI::OI() {
  js0 = new frc::Joystick(0);
  
  runButtons();
}

void OI::runButtons() {
  xBtn = new frc2::JoystickButton(js0, 2);
  xBtn->WhileHeld(rsCommand);
}

/*frc2::Command* OI::getAutonomousCommand() {
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(frc::SimpleMotorFeedForward)
}*/