// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"

using namespace DrivebaseConstants;

// This method will be called once per scheduler run
void Drivebase::Periodic() {}

void Drivebase::Drive(double xSpeed,
                      double ySpeed,
                      double rot,
                      bool fieldRelative,
                      units::second_t period) {

  // Apply slew rate limiting to xSpeed for smoother driving
  units::meters_per_second_t xSpeedLimited = 
    m_xspeedLimiter.Calculate(frc::ApplyDeadband(xSpeed, 0.02)) * kMaxSpeed;
  
  // Apply slew rate limiting to ySpeed for smoother driving
  units::meters_per_second_t ySpeedLimited =
    m_yspeedLimiter.Calculate(frc::ApplyDeadband(ySpeed, 0.02)) * kMaxSpeed;
  
  // Apply slew rate limiting to rotational speed for smoother driving
  units::radians_per_second_t rotLimited =
    m_rotLimiter.Calculate(frc::ApplyDeadband(rot, 0.02)) * kMaxAngularSpeed;

  wpi::array<frc::SwerveModuleState, 4> *states;
  if (fieldRelative) {
    wpi::array<frc::SwerveModuleState, 4> temp = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeedLimited, ySpeedLimited, rotLimited, m_gyro.GetRotation2d()),
      period));
    states = &temp;
  } else {
    wpi::array<frc::SwerveModuleState, 4> temp = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
      frc::ChassisSpeeds{xSpeedLimited, ySpeedLimited, rotLimited},
      period));
    states = &temp;
  }

  // According to WPILib, module states are not normalized, and user input can cause module speeds to go
  // above attainable max velocity.  Desaturating wheel speeds will fix this issue.  Desaturating the
  // wheel speeds makes sure all module speeds are at or below the maximum speed while maintaining the
  // ratio of speeds between modules.
  m_kinematics.DesaturateWheelSpeeds(states, kMaxSpeed);
  
  m_modules[0].SetDesiredState(states->at(0));
  m_modules[1].SetDesiredState(states->at(1));
  m_modules[2].SetDesiredState(states->at(2));
  m_modules[3].SetDesiredState(states->at(3));
}

void Drivebase::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
    wpi::array<frc::SwerveModulePosition, 4>{
        m_modules[0].GetPosition(),
        m_modules[1].GetPosition(),
        m_modules[2].GetPosition(),
        m_modules[3].GetPosition()});
}
