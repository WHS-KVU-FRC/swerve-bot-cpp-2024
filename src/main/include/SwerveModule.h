// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#ifndef _SWERVE_MODULE_H_
#define _SWERVE_MODULE_H_

#include <units/angle.h>
#include <units/length.h>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>

class SwerveModule {
 public:
  SwerveModule(int const drivePort, int const turnPort);
  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  void SetDesiredState(frc::SwerveModuleState const &state);

 private:
  frc::PWMSparkMax m_driveMotor;
  frc::PWMSparkMax m_turnMotor;
};

#endif  // #ifndef _SWERVE_MODULE_H_
