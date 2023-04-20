// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
	frc::CameraServer::StartAutomaticCapture();
	frc::SmartDashboard::PutNumber("vD:",0);
	frc::SmartDashboard::PutNumber("vX:",0);
	frc::SmartDashboard::PutNumber("vY:",0);
	frc::SmartDashboard::PutNumber("Max Speed",0.35);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
}
void Robot::AutonomousPeriodic() {
	auto angle = units::degrees_per_second_t{frc::SmartDashboard::GetNumber("vD:",0)};
	auto vx = units::meters_per_second_t{-frc::SmartDashboard::GetNumber("vX:",0)};
	auto vy = units::meters_per_second_t{frc::SmartDashboard::GetNumber("vY:",0)};
	
	m_swerve.Drive(-vx,-vy,angle,1,{0_m,0_m});
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
	if(m_Joystick.GetRawButton(3)){gyro->Reset();}

	double maxSpeed = frc::SmartDashboard::GetNumber("Max Speed", 1);

	double j_forward = m_Joystick.GetY();
	double j_strafe = -m_Joystick.GetX();
	double j_rotate = m_Joystick.GetTwist();
	double throttle = ((1 - ((m_Joystick.GetThrottle() + 1) / 2)) * maxSpeed);
	bool fieldOriented = !m_Joystick.GetRawButton(2);

	const auto forward = (-m_forwardLimiter.Calculate(frc::ApplyDeadband(j_forward, 0.13)) * throttle) * kMaxSpeed;
	const auto strafe = (-m_strafeLimiter.Calculate(frc::ApplyDeadband(j_strafe, 0.13)) * throttle) * kMaxSpeed;
	const auto rotate = (-m_rotateLimiter.Calculate(frc::ApplyDeadband(j_rotate, 0.3)) * sqrt(throttle));
	
	//Alternate centers of rotation
	int rotateAtPosition[5]{m_Joystick.GetRawButton(1),m_Joystick.GetRawButton(9),m_Joystick.GetRawButton(10),m_Joystick.GetRawButton(11),m_Joystick.GetRawButton(12)};
	auto [t, fl, fr, rl, rr] = rotateAtPosition;
	/*Tower (center of mass)*/if(t > (fl + fr + rl + rr)){centerOfRotation = kCenterOfMass;	rotation = rotate * kFastRotation;}
	/*Front Left Module     */if(fl > (t + fr + rl + rr)){centerOfRotation = {-(kRobotLength/2), +(kRobotWidth/2)}; rotation = rotate * kSlowRotation;}
	/*Front Right Module    */if(fr > (fl + t + rl + rr)){centerOfRotation = {-(kRobotLength/2), -(kRobotWidth/2)}; rotation = rotate * kSlowRotation;}
	/*Rear Left Module      */if(rl > (fl + fr + t + rr)){centerOfRotation = {+(kRobotLength/2), +(kRobotWidth/2)}; rotation = rotate * kSlowRotation;}
	/*Rear right Module     */if(rr > (fl + fr + rl + t)){centerOfRotation = {+(kRobotLength/2), -(kRobotWidth/2)}; rotation = rotate * kSlowRotation;}
	/*Robot Center          */if(1 > (fl + fr + rl + rr + t)){centerOfRotation = {0_m, 0_m}; rotation = rotate * kMediumRotation;}
	
	int c_armIn = m_Controller.GetRightBumper();
	int c_armOut = m_Controller.GetLeftBumper();
	float c_armUp = m_Controller.GetRightTriggerAxis();
	float c_armDown = m_Controller.GetLeftTriggerAxis();
	int c_intakeIn = m_Controller.GetAButton();
	int c_intakeOut = m_Controller.GetBButton();

	frc::SmartDashboard::PutNumber("throttle", throttle);
	frc::SmartDashboard::PutNumber("Angle", int(gyro->GetYaw()));
	
	m_arm.Extension(c_armIn,c_armOut);
	m_arm.Angle(c_armUp,c_armDown);
	m_arm.Intake(c_intakeIn,c_intakeOut);
	m_arm.AutoPosition(220, m_Controller.GetLeftStickButton(),m_Controller.GetRightStickButton());
	
	m_swerve.Drive(forward,strafe,rotation,fieldOriented,centerOfRotation);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRc_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif