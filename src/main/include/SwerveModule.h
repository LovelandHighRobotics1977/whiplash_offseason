// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Headers.h"

/**
 * Takes 3 CAN IDs and a magnet offset and constructs a SwerveModule object
 *
 * @param driveMotorID Drive motor CAN ID.
 * @param angleMotorID Angle motor CAN ID.
 * @param angleEncoderID Angle encoder CAN ID.
 * @param magnetOffset Magnet offset of the module's cancoder.
*/
class SwerveModule {
	public:
		
		/**
			* Takes 3 CAN IDs and a magnet offset and constructs a SwerveModule object
			*
			* @param driveMotorID Drive motor CAN ID.
			* @param angleMotorID Angle motor CAN ID.
			* @param angleEncoderID Angle encoder CAN ID.
			* @param magnetOffset Magnet offset of the module's cancoder.
		*/
		SwerveModule(int driveMotorID, int angleMotorID, int angleEncoderID, double magnetOffset);

		frc::Rotation2d getAngle();

		frc::SwerveModulePosition GetPosition(double distanceDrive) const;
		double getDrivePOS();

		/**
		 * Sets the desired state of a swerve module
		 * @param swerveModuleState State of the swerve module.
		*/
		void SetDesiredState(const frc::SwerveModuleState& state);

		frc::SwerveModuleState Optimize(const frc::SwerveModuleState& state, const frc::Rotation2d& currentAngle);

	private:
		double drivekP = 0.001;
		double drivekI = 0;
		double drivekD = 0.005;
		double drivekF = 1;

		double anglekP = 1.7;
		double anglekI = 0.0016;
		double anglekD = 160;
		double anglekF = 0;

		Gyro* gyro = Gyro::GetInstance();

		WPI_TalonFX m_driveMotor;
		WPI_TalonFX m_angleMotor;
		CANCoder m_angleEncoder;

		int CANID;
};
