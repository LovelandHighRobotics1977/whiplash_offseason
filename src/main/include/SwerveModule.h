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

class SwerveModule {
    public:
    SwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID);
    frc::SwerveModuleState GetState() const;
    frc::SwerveModulePosition GetPosition() const;
    void SetDesiredState(const frc::SwerveModuleState& state);

    private:
    static constexpr double kWheelRadius = 2;
    static constexpr int kEncoderResolution = 4096;

    static constexpr auto kModuleMaxAngularVelocity = 180_deg_per_s;       // degrees per second
    static constexpr auto kModuleMaxAngularAcceleration = 360_deg_per_s_sq; // degrees per second^2

    WPI_TalonFX m_driveMotor;
    WPI_TalonFX m_turningMotor;
    CANCoder m_turningEncoder;
};
