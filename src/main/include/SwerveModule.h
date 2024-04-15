// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#ifndef _SWERVE_MODULE_H_
#define _SWERVE_MODULE_H_

#include <units/angle.h>
#include <units/length.h>

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "Constants.h"

using namespace SwerveModuleConstants;

class SwerveModule {
 public:
  SwerveModule(int const drivePort, int const turnPort,
               int const driveEncA, int const driveEncB,
               int const turnEncA, int const turnEncB);
  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  void SetDesiredState(frc::SwerveModuleState const &desiredState);

 private:
  frc::PWMSparkMax m_driveMotor;
  frc::PWMSparkMax m_turnMotor;
  frc::Encoder m_driveEncoder;
  frc::Encoder m_turnEncoder;
  frc::PIDController m_drivePIDController{kPDrive, kIDrive, kDDrive};

  /** For TrapezoidProfile<Distance>
   * Find associated documentation here:
   * https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_trapezoid_profile.html
   * 
   * Velocity and acceleration units are built to automatically use
   * for its time units as
   * (1) units::inverse<units::seconds> for velocity, and
   * (2) units::compount_unit<Velocity, units::inverse::<units::seconds>>
   * for acceleration; i.e., the distance given to the TrapezoidProfile
   * is automatically divided by seconds and seconds^2.
   * 
   * The Constraints class within the TrapezoidProfile class takes
   * the velocity value and acceleration value in that order in
   * its constructor.
  */
  frc::ProfiledPIDController<units::radians> m_turnPIDController{
    kPTurn,
    kITurn,
    kDTurn,
    frc::TrapezoidProfile<units::radians>::Constraints{
      kMaxAngularVelocity, kMaxAngularAcceleration}};

  frc::SimpleMotorFeedforward<units::meters> m_driveFF = 
    frc::SimpleMotorFeedforward<units::meters>{kSDrive, kVDrive};

  frc::SimpleMotorFeedforward<units::radians> m_turnFF = 
    frc::SimpleMotorFeedforward<units::radians>{kSTurn, kVTurn};
};

#endif  // #ifndef _SWERVE_MODULE_H_
