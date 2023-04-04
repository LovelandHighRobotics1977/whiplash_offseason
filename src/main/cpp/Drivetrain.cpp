// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::Drive(units::feet_per_second_t xSpeed, units::feet_per_second_t ySpeed, units::degrees_per_second_t rot, bool fieldRelative) {
    auto states = m_kinematics.ToSwerveModuleStates(fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(frc::ChassisSpeeds{xSpeed, ySpeed, rot}, (ahrs->GetRotation2d())) : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

    m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    m_frontLeft.SetDesiredState(fl);
    m_frontRight.SetDesiredState(fr);
    m_rearLeft.SetDesiredState(bl);
    m_rearRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(ahrs->GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
}
