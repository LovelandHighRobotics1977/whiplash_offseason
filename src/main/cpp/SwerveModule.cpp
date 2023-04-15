// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

SwerveModule::SwerveModule(const int driveMotorID,     const int angleMotorID,       const int angleEncoderID, double magnetOffset)
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

	m_angleEncoder.ConfigMagnetOffset(magnetOffset);
	
	CANID = angleEncoderID;

	m_angleEncoder.SetPositionToAbsolute();
}

frc::SwerveModulePosition SwerveModule::GetPosition(double distanceDrive) const{
    return {units::meter_t{distanceDrive},gyro->GetRotation2d()};
}

frc::Rotation2d SwerveModule::getAngle() { 
	auto temp1 = m_angleEncoder.GetAbsolutePosition();
	auto temp2 = units::degree_t{temp1};
	auto temp3 = frc::Rotation2d{temp2};
	return temp3;
}

double SwerveModule::getDrivePOS(){
    return ((m_driveMotor.GetSelectedSensorPosition())*(5)); //sensor units multiplied by meters per sensor unit to get distance in meters
}

frc::SwerveModuleState SwerveModule::Optimize(const frc::SwerveModuleState& desiredState, const frc::Rotation2d& currentAngle) {
  auto delta = desiredState.angle - currentAngle;

	if(abs(delta.Degrees().value()) < 2){
		delta = 0_deg;
	}
  
  if (units::math::abs(delta.Degrees()) > 90_deg) {
    return {-desiredState.speed, desiredState.angle + frc::Rotation2d{180_deg}};
  } else {
    return {desiredState.speed, desiredState.angle};
  }
}

void SwerveModule::SetDesiredState(
	const frc::SwerveModuleState& desiredState) {
		// Optimize the reference state to avoid spinning further than 90 degrees
		
		auto speed = desiredState.speed;
		auto angle = desiredState.angle;
		auto const [optimized_speed, optimized_angle] = SwerveModule::Optimize(desiredState, getAngle());

		if(CANID==2){
			frc::SmartDashboard::PutNumber("Rear Left angle", optimized_angle.Degrees().value());
			frc::SmartDashboard::PutNumber("Rear Left speed", optimized_speed.value());
  		}
		if(CANID==5){
			frc::SmartDashboard::PutNumber("Front Left angle", optimized_angle.Degrees().value());
			frc::SmartDashboard::PutNumber("Front Left speed", optimized_speed.value());
  		}
		if(CANID==8){
			frc::SmartDashboard::PutNumber("Front Right angle", optimized_angle.Degrees().value());
			frc::SmartDashboard::PutNumber("Front Right speed", optimized_speed.value());
  		}
		if(CANID==11){
			frc::SmartDashboard::PutNumber("Rear Right angle", optimized_angle.Degrees().value());
			frc::SmartDashboard::PutNumber("Rear Right speed", optimized_speed.value());
  		}

		// Set the motor outputs.
		m_driveMotor.Set((double) speed);
		m_angleMotor.Set(TalonFXControlMode::Position, angle.Degrees().value()*(4096.0 / 360.0));
}