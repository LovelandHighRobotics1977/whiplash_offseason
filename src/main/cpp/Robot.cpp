// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
	frc::CameraServer::StartAutomaticCapture();

	arm_angle.SetNeutralMode(NeutralMode::Brake);
	arm_extend.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {

}
void Robot::AutonomousPeriodic() {
	m_swerve.Drive(1_fps,1_fps,units::degrees_per_second_t(0),0);
	m_swerve.UpdateOdometry();
}

void Robot::TeleopInit() {
	
}
void Robot::TeleopPeriodic() {
	double j_forward = -m_Joystick.GetY();
	double j_strafe =  m_Joystick.GetX();
	double j_rotate = -m_Joystick.GetTwist();

	double throttle = m_Joystick.GetThrottle();
	throttle += 1;
	throttle = throttle/2;

	const auto forward = -m_forwardLimiter.Calculate(frc::ApplyDeadband(j_forward, 0.02)) * Drivetrain::kMaxSpeed;
	const auto strafe = -m_strafeLimiter.Calculate(frc::ApplyDeadband(j_strafe, 0.02)) * Drivetrain::kMaxSpeed;
	const auto rotate = -m_rotateLimiter.Calculate(frc::ApplyDeadband(j_rotate, 0.02)) * Drivetrain::kMaxAngularSpeed;

	m_swerve.Drive(forward,strafe,rotate,1);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif