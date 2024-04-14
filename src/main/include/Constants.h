
#pragma once
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <numbers>

#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

using namespace units::literals;

namespace SwerveModuleConstants {

static units::volt_t constexpr kSDrive = 1_V; // kS feedforward
static auto constexpr kVDrive = 3_V / 1_mps;  // kV feedforward
static double constexpr kPDrive = 1.0;        // kP PID feedback
static double constexpr kIDrive = 0.0;        // kI PID feedback
static double constexpr kDDrive = 0.0;        // kD PID feedback
static units::volt_t constexpr kSTurn = 1_V;  // likewise for turn motor
static auto constexpr kVTurn = 0.5_V / 1_rad_per_s;
static double constexpr kPTurn = 1.0;
static double constexpr kITurn = 0.0;
static double constexpr kDTurn = 0.0;

static units::meter_t constexpr kWheelRadius = 0.0508_m;
static int constexpr kEncoderResolution = 4096;

// used for trapezoidal profiling for turn PID controller
static units::radians_per_second_t constexpr kMaxAngularVelocity =
  std::numbers::pi * 1_rad_per_s;
static units::radians_per_second_squared_t constexpr kMaxAngularAcceleration =
  std::numbers::pi * 2_rad_per_s_sq;

} // SwerveModuleConstants

#endif  // #ifndef _CONSTANTS_H_