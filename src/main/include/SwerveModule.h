// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
/**
 * Represents a single swerve module in a swerve drivetrain
*/
class SwerveModule {
	public:
		/**
			* Takes 3 CAN IDs and constructs a SwerveModule object
			*
			* @param driveMotorID Drive motor CAN ID.
			* @param angleMotorID Angle motor CAN ID.
			* @param driveEncoderID Angle encoder CAN ID.
		*/
		SwerveModule(int driveMotorID, int angleMotorID, int angleEncoderID);
		frc::SwerveModuleState GetState() const;
		frc::SwerveModulePosition GetPosition() const;
		void SetDesiredState(const frc::SwerveModuleState& state);

	private:
		double kWheelRadius = 2;
		int kEncoderResolution = 4096;
		
		units::feet_per_second_t kMaxDistancePerSec = 3.0_fps;  //maximum travel distance in feet per second
		static constexpr auto kModuleMaxAngularVelocity = 180_deg_per_s;        // degrees per second
		static constexpr auto kModuleMaxAngularAcceleration = 360_deg_per_s_sq; // degrees per second^2

		double drivekP = 0.001;
		double drivekI = 0;
		double drivekD = 0.005;
		double drivekF = 1;
		

		double anglekP = 1.7;
		double anglekI = 0.0016;
		double anglekD = 160;
		double anglekF = 0;
		

		WPI_TalonFX m_driveMotor;
		WPI_TalonFX m_angleMotor;
		CANCoder m_angleEncoder;
};
