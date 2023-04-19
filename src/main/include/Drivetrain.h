// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 * 
 */
class Drivetrain {
	public:
		Drivetrain(){
			gyro->Reset();
		}
		
		
		/**
		 * Drives the swerve robot
		 * @param forward Forward movement of the robot in meters/sec.
		 * @param strafe Sideways movement of the robot in meters/sec.
		 * @param rotate Rotational movement of the robot in degrees/sec
		 * @param fieldRelative Is the robot being driven field oriented?
		 * @param speedySpin Is the robot spinning quick?
		*/
		void Drive(units::meters_per_second_t forward, units::meters_per_second_t strafe, units::degrees_per_second_t rotate, bool fieldRelative, frc::Translation2d centerOfRotation);
		/**
		 * Updates the swerve drive odometry
		 * @param robotAngle the angle of the robot as a rotation2D
		 * @return returns a pose2d of the robot's position on the field.
		*/
		void UpdateOdometry();
		
		static constexpr units::meters_per_second_t kMaxSpeed = 1.0_mps;  // 3 meters per second
		

	private:
		Gyro* gyro = Gyro::GetInstance();

		//gear ratio is L2 6.75:1

		// This is not how it should be but doing it "correctly" (-+,++,+-,--) causes
		// the wheels to form an "X" instead of diamond while turning.
		// The x coordinate is the inverse of the correct X coordinate
		// It's wrong but (++,-+,--,+-) it works, no touchy.
		
		SwerveModule m_rearLeft{0, 1, 2, 110};
		frc::Translation2d m_rearLeftLocation{+0.3048_m, +0.3048_m};
		
		SwerveModule m_frontLeft{3, 4, 5, 148};
		frc::Translation2d m_frontLeftLocation{-0.3048_m, +0.3048_m};
		
		SwerveModule m_frontRight{6, 7, 8, 250};
		frc::Translation2d m_frontRightLocation{-0.3048_m, -0.3048_m};

		SwerveModule m_rearRight{9, 10, 11, 105};
		frc::Translation2d m_rearRightLocation{+0.3048_m, -0.3048_m};

		frc::SwerveDriveKinematics<4> m_kinematics{m_rearLeftLocation, m_frontLeftLocation, m_frontRightLocation, m_rearRightLocation};

		frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, 
												gyro->GetRotation2d(),
												{
											   		m_rearLeft.GetPosition(),  m_frontLeft.GetPosition(), 
													m_frontRight.GetPosition(), m_rearRight.GetPosition()
												},
												frc::Pose2d{0_m,0_m,0_deg}
											};
};
