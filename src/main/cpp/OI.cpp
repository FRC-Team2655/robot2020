/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"

using namespace team2655;

OI::OI() {
  js0 = new frc::Joystick(0);

  frc::JoystickButton *xBtn = new frc::JoystickButton(js0, 2);
  frc::JoystickButton *squareBtn = new frc::JoystickButton(js0, 1);
  frc::JoystickButton *r3Btn = new frc::JoystickButton(js0, 12);
  frc::JoystickButton *r2Btn = new frc::JoystickButton(js0, 8);
  frc::JoystickButton *l1Btn = new frc::JoystickButton(js0, 5);
  frc::JoystickButton *r1Btn = new frc::JoystickButton(js0, 6);
  frc::JoystickButton *triangleBtn = new frc::JoystickButton(js0, 4);
  frc::JoystickButton *circleBtn = new frc::JoystickButton(js0, 3);
  frc::JoystickButton *shareBtn = new frc::JoystickButton(js0, 9);
  frc::JoystickButton *l2Btn = new frc::JoystickButton(js0, 7);
}
