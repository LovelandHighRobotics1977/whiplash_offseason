// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
  //****************************************************************    Various Miscellanious Configurations
  //frc::CameraServer::StartAutomaticCapture();

  m_rearRightAngle.SetSensorPhase(true);
  m_frontRightAngle.SetSensorPhase(true);
  m_rearLeftAngle.SetSensorPhase(true);
  m_frontLeftAngle.SetSensorPhase(true);
  m_rearRightAngle.SetNeutralMode(NeutralMode::Brake);
  m_frontRightAngle.SetNeutralMode(NeutralMode::Brake);
  m_rearLeftAngle.SetNeutralMode(NeutralMode::Brake);
  m_frontRightAngle.SetNeutralMode(NeutralMode::Brake);

  m_rearRightDrive.SetNeutralMode(NeutralMode::Brake);
  m_frontRightDrive.SetNeutralMode(NeutralMode::Brake);
  m_rearLeftDrive.SetNeutralMode(NeutralMode::Brake);
  m_frontRightDrive.SetNeutralMode(NeutralMode::Brake);

  arm_angle.SetNeutralMode(NeutralMode::Brake);
  arm_extend.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  p_solenoidA.Set(frc::DoubleSolenoid::Value::kForward);
  /*m_rearRightDrive.SetNeutralMode(NeutralMode::Brake);
  m_frontRightDrive.SetNeutralMode(NeutralMode::Brake);
  m_rearLeftDrive.SetNeutralMode(NeutralMode::Brake);
  m_frontRightDrive.SetNeutralMode(NeutralMode::Brake);*/

  //****************************************************************    Configure Rear Right Angle Motor
  m_rearRightAngle.ConfigFactoryDefault();
  m_rearRightAngle.ConfigRemoteFeedbackFilter(11, RemoteSensorSource(13), 0, 0);
  m_rearRightAngle.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0

  m_rearRightAngle.Config_kP(0, 1.7);
  m_rearRightAngle.Config_kI(0, .0016);
  m_rearRightAngle.Config_kD(0, 160);
  m_rearRightAngle.Config_kF(0, 0);
  m_rearRightAngle.Config_IntegralZone(0, 20);

  m_rearRightAngle.ConfigNominalOutputForward(0);
	m_rearRightAngle.ConfigNominalOutputReverse(0);
	m_rearRightAngle.ConfigPeakOutputForward(1);
	m_rearRightAngle.ConfigPeakOutputReverse(-1);

  //****************************************************************    Configure Front Right Angle Motor
  m_frontRightAngle.ConfigFactoryDefault();
  m_frontRightAngle.ConfigRemoteFeedbackFilter(5, RemoteSensorSource(13), 0, 0);
  m_frontRightAngle.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0

  m_frontRightAngle.Config_kP(0, 1.7);
  m_frontRightAngle.Config_kI(0, .0016);
  m_frontRightAngle.Config_kD(0, 160);
  m_frontRightAngle.Config_kF(0, 0);
  m_frontRightAngle.Config_IntegralZone(0, 20);

  m_frontRightAngle.ConfigNominalOutputForward(0);
	m_frontRightAngle.ConfigNominalOutputReverse(0);
	m_frontRightAngle.ConfigPeakOutputForward(1);
	m_frontRightAngle.ConfigPeakOutputReverse(-1);

  //****************************************************************    Configure Rear Left Angle Motor
  m_rearLeftAngle.ConfigFactoryDefault();
  m_rearLeftAngle.ConfigRemoteFeedbackFilter(8, RemoteSensorSource(13), 0, 0);
  m_rearLeftAngle.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0

  m_rearLeftAngle.Config_kP(0, 1.7);
  m_rearLeftAngle.Config_kI(0, .0016);
  m_rearLeftAngle.Config_kD(0, 160);
  m_rearLeftAngle.Config_kF(0, 0);
  m_rearLeftAngle.Config_IntegralZone(0, 20);

  m_rearLeftAngle.ConfigNominalOutputForward(0);
	m_rearLeftAngle.ConfigNominalOutputReverse(0);
	m_rearLeftAngle.ConfigPeakOutputForward(1);
	m_rearLeftAngle.ConfigPeakOutputReverse(-1);

  //****************************************************************    Configure Front Left Angle Motor
  m_frontLeftAngle.ConfigFactoryDefault();
  m_frontLeftAngle.ConfigRemoteFeedbackFilter(2, RemoteSensorSource(13), 0, 0);
  m_frontLeftAngle.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0

  m_frontLeftAngle.Config_kP(0, 1.7);
  m_frontLeftAngle.Config_kI(0, .0016);
  m_frontLeftAngle.Config_kD(0, 160);
  m_frontLeftAngle.Config_kF(0, 0);
  m_frontLeftAngle.Config_IntegralZone(0, 20);

  m_frontLeftAngle.ConfigNominalOutputForward(0);
	m_frontLeftAngle.ConfigNominalOutputReverse(0);
	m_frontLeftAngle.ConfigPeakOutputForward(1);
	m_frontLeftAngle.ConfigPeakOutputReverse(-1);

  //*****************************************************************    Configure Cancoder Magnet Offsets
  m_frontLeftSensor.ConfigMagnetOffset(105);
  m_frontRightSensor.ConfigMagnetOffset(110);
  m_rearLeftSensor.ConfigMagnetOffset(250);
  m_rearRightSensor.ConfigMagnetOffset(148);
  m_frontLeftSensor.SetPositionToAbsolute();
  m_frontRightSensor.SetPositionToAbsolute();
  m_rearLeftSensor.SetPositionToAbsolute();
  m_rearRightSensor.SetPositionToAbsolute();
  
  //-*****************************************************************    Configure Rear Right Drive Motor
  m_rearRightDrive.ConfigFactoryDefault();
  m_rearRightDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
  m_rearRightDrive.Config_kP(0, drivekP);
  m_rearRightDrive.Config_kI(0, drivekI);
  m_rearRightDrive.Config_kD(0, drivekD);
  m_rearRightDrive.Config_kF(0, drivekF);
  m_rearRightDrive.ConfigNominalOutputForward(0);
	m_rearRightDrive.ConfigNominalOutputReverse(0);
	m_rearRightDrive.ConfigPeakOutputForward(1);
	m_rearRightDrive.ConfigPeakOutputReverse(-1);
  //-*****************************************************************    Configure Rear Left Drive Motor
  m_rearLeftDrive.ConfigFactoryDefault();
  m_rearLeftDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
  m_rearLeftDrive.Config_kP(0, drivekP);
  m_rearLeftDrive.Config_kI(0, drivekI);
  m_rearLeftDrive.Config_kD(0, drivekD);
  m_rearLeftDrive.Config_kF(0, drivekF);
  m_rearLeftDrive.ConfigNominalOutputForward(0);
	m_rearLeftDrive.ConfigNominalOutputReverse(0);
	m_rearLeftDrive.ConfigPeakOutputForward(1);
	m_rearLeftDrive.ConfigPeakOutputReverse(-1);
  //-*****************************************************************    Configure Front Right Drive Motor
  m_frontRightDrive.ConfigFactoryDefault();
  m_frontRightDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
  m_frontRightDrive.Config_kP(0, drivekP);
  m_frontRightDrive.Config_kI(0, drivekI);
  m_frontRightDrive.Config_kD(0, drivekD);
  m_frontRightDrive.Config_kF(0, drivekF);
  m_frontRightDrive.ConfigNominalOutputForward(0);
	m_frontRightDrive.ConfigNominalOutputReverse(0);
	m_frontRightDrive.ConfigPeakOutputForward(1);
	m_frontRightDrive.ConfigPeakOutputReverse(-1);
  //-*****************************************************************    Configure Front Left Drive Motor
  m_frontLeftDrive.ConfigFactoryDefault();
  m_frontLeftDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
  m_frontLeftDrive.Config_kP(0, drivekP);
  m_frontLeftDrive.Config_kI(0, drivekI);
  m_frontLeftDrive.Config_kD(0, drivekD);
  m_frontLeftDrive.Config_kF(0, drivekF);
  m_frontLeftDrive.ConfigNominalOutputForward(0);
	m_frontLeftDrive.ConfigNominalOutputReverse(0);
	m_frontLeftDrive.ConfigPeakOutputForward(1);
	m_frontLeftDrive.ConfigPeakOutputReverse(-1);
  //-****************************************************************    Configure the gyro board
  Robot::ahrs = new AHRS(frc::I2C::Port::kMXP);
  
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  
}
void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
  
}

void Robot::TeleopPeriodic() {
  double j_forward = -m_Joystick.GetY();
  double j_strafe =  m_Joystick.GetX();
  double j_rotate = -m_Joystick.GetTwist();

  if((j_forward < 0.2) && (j_forward > -0.2)){j_forward = 0;}
  if(( j_strafe < 0.2) && ( j_strafe > -0.2)){ j_strafe = 0;}
  if(( j_rotate < 0.2) && ( j_rotate > -0.2)){ j_rotate = 0;}

  double throttle = m_Joystick.GetThrottle();
  throttle += 1;
  throttle = throttle/2;
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