// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#ifndef _SUBSYSTEMS_DRIVEBASE_H_
#define _SUBSYSTEMS_DRIVEBASE_H_

#include <numbers>


#include <frc/AnalogGyro.h>
#include <frc/MathUtil.h> // frc::ApplyDeadband()
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/array.h>

#include "Constants.h"
#include "SwerveModule.h"

using namespace DrivebaseConstants;

class Drivebase : public frc2::SubsystemBase {
 public:
  Drivebase() { m_gyro.Reset(); }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Drive(double xSpeed,
             double ySpeed,
             double rot,
             bool fieldRelative,
             units::second_t period);
  void UpdateOdometry();
 private:
  // Note that relative to the robot, +x is up, and +y is left
  frc::Translation2d m_topLeft  = frc::Translation2d{ kWheelbase/2,  kTrackwidth/2};
  frc::Translation2d m_topRight = frc::Translation2d{ kWheelbase/2, -kTrackwidth/2};
  frc::Translation2d m_botLeft  = frc::Translation2d{-kWheelbase/2,  kTrackwidth/2};
  frc::Translation2d m_botRight = frc::Translation2d{-kWheelbase/2, -kTrackwidth/2};

  SwerveModule m_modules[4] = {
    SwerveModule{1, 2, 0, 1, 2, 3},       // top left
    SwerveModule{3, 4, 4, 5, 6, 7},       // top right
    SwerveModule{5, 6, 8, 9, 10, 11},     // bottom left
    SwerveModule{7, 8, 12, 13, 14, 15}};  // bottom right
  
  frc::AnalogGyro m_gyro{0};

  // Specific order doesn't matter as long as you're consistent everywhere
  frc::SwerveDriveKinematics<4> m_kinematics = 
    frc::SwerveDriveKinematics<4>{
      m_topLeft,
      m_topRight,
      m_botLeft,
      m_botRight};
  
  frc::SwerveDriveOdometry<4> m_odometry = 
    frc::SwerveDriveOdometry<4>{
      m_kinematics,
      m_gyro.GetRotation2d(),
      wpi::array<frc::SwerveModulePosition, 4>{
        m_modules[0].GetPosition(),
        m_modules[1].GetPosition(),
        m_modules[2].GetPosition(),
        m_modules[3].GetPosition()}};
  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
};

#endif  // #ifndef _SUBSYSTEMS_DRIVEBASE_H_