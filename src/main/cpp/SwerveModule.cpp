// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>
SwerveModule::SwerveModule(const int driveMotorID,     const int angleMotorID,       const int angleEncoderID)
					  : m_driveMotor{driveMotorID}, m_angleMotor{angleMotorID}, m_angleEncoder{angleEncoderID} {
	//--------

	m_driveMotor.SetNeutralMode(NeutralMode::Brake);

	m_driveMotor.ConfigFactoryDefault();
	m_driveMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
	m_driveMotor.Config_kP(0, drivekP);
	m_driveMotor.Config_kI(0, drivekI);
	m_driveMotor.Config_kD(0, drivekD);
	m_driveMotor.Config_kF(0, drivekF);
	m_driveMotor.ConfigNominalOutputForward(0);
	m_driveMotor.ConfigNominalOutputReverse(0);
	m_driveMotor.ConfigPeakOutputForward(1);
	m_driveMotor.ConfigPeakOutputReverse(-1);

	//--------

	m_angleMotor.SetSensorPhase(true);
	m_angleMotor.SetNeutralMode(NeutralMode::Brake);

	m_angleMotor.ConfigFactoryDefault();
	m_angleMotor.ConfigRemoteFeedbackFilter(angleEncoderID, RemoteSensorSource(13), 0, 0);
	m_angleMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0
	m_angleMotor.Config_kP(0, anglekP);
	m_angleMotor.Config_kI(0, anglekI);
	m_angleMotor.Config_kD(0, anglekD);
	m_angleMotor.Config_kF(0, anglekF);
	m_angleMotor.Config_IntegralZone(0, 20);
	m_angleMotor.ConfigNominalOutputForward(0);
	m_angleMotor.ConfigNominalOutputReverse(0);
	m_angleMotor.ConfigPeakOutputForward(1);
	m_angleMotor.ConfigPeakOutputReverse(-1);

	//--------

	switch (angleEncoderID){
	case 2: //front left
		m_angleEncoder.ConfigMagnetOffset(105);
		break;
	case 5: //front right
		m_angleEncoder.ConfigMagnetOffset(110);
		break;
	case 8: //rear left
		m_angleEncoder.ConfigMagnetOffset(250);
		break;
	case 11: //rear right
		m_angleEncoder.ConfigMagnetOffset(148);
		break;
	}
	m_angleEncoder.SetPositionToAbsolute();
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  //m_driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius / kEncoderResolution);

  // Set the distance (in this case, angle) per pulse for the angle encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  //m_angleEncoder.SetDistancePerPulse(2 * std::numbers::pi / kEncoderResolution);
}
/*
frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{m_driveMotor.Get()}, units::degree_t{m_angleEncoder.GetPosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meters_t{m_driveEncoder.GetDistance()}, units::degree_t{m_angleEncoder.GetDistance()}};
}
*/
void SwerveModule::SetDesiredState(
	const frc::SwerveModuleState& referenceState) {
		// Optimize the reference state to avoid spinning further than 90 degrees
		const auto state = frc::SwerveModuleState::Optimize(referenceState, units::degree_t{m_angleEncoder.GetPosition()});

		// Calculate the drive output
		auto moduleSpeed = (units::feet_per_second_t(state.speed));
		const auto driveOutput = units::feet_per_second_t(moduleSpeed.value()/kMaxDistancePerSec.value());
		// Calculate the angle motor output
		const auto turnOutput = state.angle.Degrees();

		// Set the motor outputs.
		m_driveMotor.Set(TalonFXControlMode::Velocity, driveOutput.value());
		m_angleMotor.Set(TalonFXControlMode::Position, turnOutput.value());
}
