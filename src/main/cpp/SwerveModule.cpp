// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "SwerveModule.h"

using namespace SwerveModuleConstants;

SwerveModule::SwerveModule(int const drivePort, int const turnPort)
  : m_driveMotor{drivePort},
    m_turnMotor{turnPort} {}