// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::Drive(units::meters_per_second_t forward, units::meters_per_second_t strafe, units::degrees_per_second_t rotate, bool fieldRelative) {
	auto states = m_kinematics.ToSwerveModuleStates(fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(frc::ChassisSpeeds{forward, strafe, rotate}, ahrs->GetRotation2d()) 
																  : frc::ChassisSpeeds{forward, strafe, rotate});

	m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

	auto [fl, fr, rl, rr] = states;

	m_frontLeft.SetDesiredState(fl);
	m_frontRight.SetDesiredState(fr);
	m_rearLeft.SetDesiredState(rl);
	m_rearRight.SetDesiredState(rr);
}

void Drivetrain::UpdateOdometry() {m_odometry.Update(ahrs->GetRotation2d(), 
													{
														m_frontLeft.GetPosition(m_frontLeft.getDrivePOS()), m_frontRight.GetPosition(m_frontRight.getDrivePOS()), 
														m_rearLeft.GetPosition(m_rearLeft.getDrivePOS()),  m_rearRight.GetPosition(m_rearRight.getDrivePOS())
													});
}
