// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
	frc::CameraServer::StartAutomaticCapture();
	m_Controller.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {

}
void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
	m_Controller.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1);
}
void Robot::TeleopPeriodic() {
	m_Controller.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
	double j_forward = m_Joystick.GetY();
	double j_strafe = -m_Joystick.GetX();
	double j_rotate = -m_Joystick.GetTwist();
	double throttle = ((-m_Joystick.GetThrottle() + 1) / 2);

	const auto forward = (-m_forwardLimiter.Calculate(frc::ApplyDeadband(j_forward, 0.2)) * throttle) * Drivetrain::kMaxSpeed;
	const auto strafe = (-m_strafeLimiter.Calculate(frc::ApplyDeadband(j_strafe, 0.2)) * throttle) * Drivetrain::kMaxSpeed;
	const auto rotate = (-m_rotateLimiter.Calculate(frc::ApplyDeadband(j_rotate, 0.2)) * throttle) * units::degrees_per_second_t{180};

	int c_armIn = m_Controller.GetLeftBumper();
	int c_armOut = m_Controller.GetLeftBumper();
	float c_armUp = m_Controller.GetRightTriggerAxis();
	float c_armDown = m_Controller.GetLeftTriggerAxis();
	int c_intakeIn = m_Controller.GetAButton();
	int c_intakeOut = m_Controller.GetBButton();
	bool c_direction = m_Controller.GetLeftStickButton();
	bool c_enabled = m_Controller.GetRightStickButton();
	
	m_swerve.Drive(forward,strafe,rotate,1);

	m_arm.Extension(c_armIn,c_armOut);
	m_arm.Angle(c_armUp,c_armDown);
	m_arm.Intake(c_intakeIn,c_intakeOut);
	m_arm.AutoPosition(220,c_direction,c_enabled);
}

void Robot::DisabledInit() {
	m_Controller.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif