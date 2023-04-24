// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::Drive(units::meters_per_second_t forward, units::meters_per_second_t strafe, units::degrees_per_second_t rotate, bool fieldRelative, frc::Translation2d centerOfRotation) {
	auto states = m_kinematics.ToSwerveModuleStates(fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
													frc::ChassisSpeeds{forward, strafe, rotate}, gyro->GetRotation2d()) 
												  : frc::ChassisSpeeds{forward, strafe, rotate},
												  centerOfRotation);

	m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

	auto [rl, fl, fr, rr] = states;

	m_rearLeft.SetDesiredState(rl);
	m_frontLeft.SetDesiredState(fl);
	m_frontRight.SetDesiredState(fr);
	m_rearRight.SetDesiredState(rr);
	
}

frc::Pose2d Drivetrain::UpdateOdometry() {return m_odometry.Update(gyro->GetRotation2d(), 
																{
																	m_rearLeft.GetPosition(), m_frontLeft.GetPosition(), 
																	m_frontRight.GetPosition(), m_rearRight.GetPosition()
																});
}

void Drivetrain::ResetOdometry() {
	m_odometry.ResetPosition(gyro->GetRotation2d(), 
							{
								m_rearLeft.GetPosition(), m_frontLeft.GetPosition(), 
								m_frontRight.GetPosition(), m_rearRight.GetPosition()
							},
							frc::Pose2d{0_m,0_m,0_deg});
}
