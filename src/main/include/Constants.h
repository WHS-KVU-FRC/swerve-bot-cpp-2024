
#pragma once
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <numbers>

#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>

using namespace units::literals;

namespace SwerveModuleConstants {
  static units::meter_t constexpr kWheelRadius = 0.0508_m;
  static units::radians_per_second_t constexpr kMaxAngularVelocity =
    std::numbers::pi * 1_rad_per_s;
  static units::radians_per_second_squared_t constexpr kMaxAngularAcceleration =
    std::numbers::pi * 2_rad_per_s_sq;
}

#endif  // #ifndef _CONSTANTS_H_