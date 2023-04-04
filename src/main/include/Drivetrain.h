// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include "AHRS.h"
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "SwerveModule.h"

#include "Robot.h"

/**
 * Represents a swerve drive style drivetrain.
 * 
 */
class Drivetrain {
	public:
		AHRS *ahrs;
		Drivetrain() { ahrs->Reset(); }

		/**
		 * Drives the swerve robot
		 * @param forward Forward movement of the robot in feet/sec.
		 * @param strafe Sideways movement of the robot in feet/sec.
		 * @param rotate Rotational movement of the robot in degrees/sec
		 * @param robotAngle Angle of the robot as a rotation2D.
		 * @param fieldRelative Is the robot being driven field oriented?
		 * 
		*/
		void Drive(units::feet_per_second_t forward, units::feet_per_second_t strafe, units::degrees_per_second_t rotate, auto robotAngle, bool fieldRelative);
		/**
		 * Updates the swerve drive odometry
		 * @param robotAngle the angle of the robot as a rotation2D
		*/
		void UpdateOdometry(auto robotAngle);

		static constexpr units::feet_per_second_t kMaxSpeed = 3.0_fps;  // 3 feet per second
		static constexpr units::degrees_per_second_t kMaxAngularSpeed{180};  // 1/2 rotation per second

	private:
		frc::Translation2d m_frontLeftLocation{+0.3048_m, +0.3048_m};
		frc::Translation2d m_frontRightLocation{+0.3048_m, -0.3048_m};
		frc::Translation2d m_rearLeftLocation{-0.3048_m, +0.3048_m};
		frc::Translation2d m_rearRightLocation{-0.3048_m, -0.3048_m};

		SwerveModule m_frontLeft{0, 1, 2};
		SwerveModule m_frontRight{3, 4, 5};
		SwerveModule m_rearLeft{6, 7, 8};
		SwerveModule m_rearRight{9, 10, 11};
  

		frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_rearLeftLocation, m_rearRightLocation};

		frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, ahrs->GetRotation2d(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_rearLeft.GetPosition(), m_rearRight.GetPosition()}};
};
