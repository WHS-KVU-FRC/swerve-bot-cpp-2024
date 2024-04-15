// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  m_drive.SetDefaultCommand(
    frc2::cmd::Run([this] {m_drive.Drive(
      -m_controller.GetLeftY(),  // As +x is up, and up on controller is -y
      -m_controller.GetLeftX(),  // As +y is left, and left on controller is -x
      -m_controller.GetRightX(), // As CCW is positive, and left on controller is -x
      true,     // isFieldOriented
      20_ms);}, // default time between roboRIO periodic cycles
    {&m_drive})
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
