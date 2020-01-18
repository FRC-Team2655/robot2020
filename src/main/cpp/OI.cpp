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

  frc2::JoystickButton *xBtn = new frc2::JoystickButton(js0, 2);
  frc2::JoystickButton *squareBtn = new frc2::JoystickButton(js0, 1);
  frc2::JoystickButton *r3Btn = new frc2::JoystickButton(js0, 12);
  frc2::JoystickButton *r2Btn = new frc2::JoystickButton(js0, 8);
  frc2::JoystickButton *l1Btn = new frc2::JoystickButton(js0, 5);
  frc2::JoystickButton *r1Btn = new frc2::JoystickButton(js0, 6);
  frc2::JoystickButton *triangleBtn = new frc2::JoystickButton(js0, 4);
  frc2::JoystickButton *circleBtn = new frc2::JoystickButton(js0, 3);
  frc2::JoystickButton *shareBtn = new frc2::JoystickButton(js0, 9);
  frc2::JoystickButton *l2Btn = new frc2::JoystickButton(js0, 7);
}

