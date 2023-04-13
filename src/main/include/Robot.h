// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Drivetrain.h"
#include "Arm.h"

class Robot : public frc::TimedRobot {
	public:

		void RobotInit() override;
		void RobotPeriodic() override;

		void AutonomousInit() override;
		void AutonomousPeriodic() override;

		void TeleopInit() override;
		void TeleopPeriodic() override;

		void DisabledInit() override;
		void DisabledPeriodic() override;

		void TestInit() override;
		void TestPeriodic() override;

	private:
		frc::Joystick m_Joystick{0};
		frc::XboxController m_Controller{1};

		//Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1
		frc::SlewRateLimiter<units::dimensionless::scalar> m_forwardLimiter{3 / 1_s};
		frc::SlewRateLimiter<units::dimensionless::scalar> m_strafeLimiter{3 / 1_s};
		frc::SlewRateLimiter<units::dimensionless::scalar> m_rotateLimiter{3 / 1_s};

		frc::Timer timer;

		Drivetrain m_swerve{};

		Arm m_arm{};

		double conversionFactor = 4096.0/ 360.0;

		double yaw;

		int resetTimer = 0;
	};
