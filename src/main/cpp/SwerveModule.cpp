// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/geometry/Rotation2d.h>

#include "Constants.h"
#include "SwerveModule.h"

using namespace SwerveModuleConstants;

SwerveModule::SwerveModule(int const drivePort, int const turnPort,
                           int const driveEncA, int const driveEncB,
                           int const turnEncA, int const turnEncB)
  : m_driveMotor{drivePort},
    m_turnMotor{turnPort},
    m_driveEncoder{driveEncA, driveEncB},
    m_turnEncoder{turnEncA, turnEncB} {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.  Distance traveled in one rotation of the wheel is the
  // circumference of the wheel (i.e., C = 2 pi r)
  m_driveEncoder.SetDistancePerPulse(2*std::numbers::pi * kWheelRadius.value() /
                                     kEncoderResolution);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  m_turnEncoder.SetDistancePerPulse(2*std::numbers::pi / kEncoderResolution);
  
  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turnPIDController.EnableContinuousInput(
    -units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return frc::SwerveModuleState{
    units::meters_per_second_t{m_driveEncoder.GetRate()},
    units::radian_t{m_turnEncoder.GetDistance()}
  };
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return frc::SwerveModulePosition{
    units::meter_t{m_driveEncoder.GetDistance()},
    units::radian_t{m_turnEncoder.GetDistance()}
  };
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState const &desiredState) {
  frc::Rotation2d const turnEncoderRotation = 
    frc::Rotation2d{units::radian_t{m_turnEncoder.GetDistance()}};
  
  frc::SwerveModuleState optimizedState = 
    frc::SwerveModuleState::Optimize(desiredState, turnEncoderRotation);
  
  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  // Note that both optimizedState.angle and turnEncoderRotation are
  // frc::Rotation2d objects, and that the "minus" (-) operator for that
  // class has been overridden.
  optimizedState.speed *= (optimizedState.angle - turnEncoderRotation).Cos();

  // Calculate the drive output from the drive PID controller.
  units::volt_t const driveOutput = units::volt_t{m_drivePIDController.Calculate(
    m_driveEncoder.GetRate(), optimizedState.speed.value())
  };
  units::volt_t const driveFeedforward = m_driveFF.Calculate(optimizedState.speed);

  // Calculate the turn output from the turn PID controller
  units::volt_t const turnOutput = units::volt_t{m_turnPIDController.Calculate(
    units::radian_t{m_turnEncoder.GetRate()}, optimizedState.angle.Radians())
  };

  // Note that the setpoint velocity is needed for turning PID control
  // since we don't want to instantly go to the optimizedState's turn velocity.
  // We actually want to use the setpoint given by the trapezoidal profile.
  units::volt_t const turnFeedforward = m_turnFF.Calculate(
    m_turnPIDController.GetSetpoint().velocity
  );

  // Set the motor outputs via FF + PID
  m_driveMotor.SetVoltage(driveFeedforward + driveOutput);
  m_turnMotor.SetVoltage(turnFeedforward + turnOutput);
}