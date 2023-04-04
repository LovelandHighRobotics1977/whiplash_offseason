// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>
SwerveModule::SwerveModule(const int driveMotorID,
                           const int turningMotorID,
                           const int turningEncoderID)
    : m_driveMotor{driveMotorID},
      m_turningMotor{turningMotorID},
      m_turningEncoder{turningEncoderID} {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  //m_driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius / kEncoderResolution);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  //m_turningEncoder.SetDistancePerPulse(2 * std::numbers::pi / kEncoderResolution);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  //m_turningPIDController.EnableContinuousInput(-units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{m_driveMotor.Get()}, units::degree_t{m_turningEncoder.GetPosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meters_t{m_driveEncoder.GetDistance()}, units::degree_t{m_turningEncoder.GetDistance()}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        const auto state = frc::SwerveModuleState::Optimize(referenceState, units::degree_t{m_turningEncoder.GetPosition()/4096});

        // Calculate the drive output from the drive PID controller.
        const auto driveOutput = (units::feet_per_second_t(state.speed.value()));
        // Calculate the turning motor output from the turning PID controller.
        const auto turnOutput = state.angle.Degrees();

        // Set the motor outputs.
        m_driveMotor.Set(TalonFXControlMode::Velocity, driveOutput);
        m_turningMotor.Set(TalonFXControlMode::Position, int(turnOutput));
}
