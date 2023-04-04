// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <math.h>

#include "unitincludes.h"

#include <ctre/Phoenix.h>
#include "rev/CANSparkMax.h"

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <cameraserver/CameraServer.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

#include "AHRS.h"

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

  //Controls the positioning of the mechanism arm
  void GrabberArm(int out, int in, float down, float up, bool retract){
    if(retract == false){
    if((!armSwitch.Get())&&(out == 0)&&(in == 1)){
      arm_extend.Set(1);
    }
    else if((out == 1)&&(in == 0)){
      arm_extend.Set(-1);
    }else {
      arm_extend.Set(0);
    }

    if((down>0) && (up==0)){
      arm_angle.Set(.3 * down);
    }else if((up>0) && (down==0)&&(!insideSwitch.Get())){
      arm_angle.Set(-.3 * up);
    }else{
      arm_angle.Set(0);
    }
    }

    if(retract == true){
      if(!armSwitch.Get()){
        arm_extend.Set(1);
      }else{
        arm_extend.Set(0);
      }

      if((!insideSwitch.Get())&&(armSwitch.Get())){
        arm_angle.Set(-.2);
      }else{
        arm_angle.Set(0);
      }

    }
  }

  //Automatically moves the arm up to the scoring position.
  void AutoRaiseArm(int position, bool enabled){
    if(!enabled){
      positionSpeed = .3;
    }
    if(enabled){
    if(abs(arm_encoder.GetDistance()) < position-2){
      arm_angle.Set(positionSpeed);
    }else if(abs(arm_encoder.GetDistance()) > (position+2)){
      arm_angle.Set(-positionSpeed);
    }else if((abs(arm_encoder.GetDistance()) > (position-2))&&(abs(arm_encoder.GetDistance()) < (position+2))){
      positionSpeed = .05;
      arm_angle.Set(0);
    }
    }
  }

  //Controls the roller intakes to pick up or place game pieces
  void Intake(int in, int out){
    m_intake.Set(in-out);
  }

  private:
    frc::Joystick m_Joystick{0};
    frc::XboxController m_Controller{1};

    WPI_TalonFX m_frontLeftAngle{0};
    WPI_TalonFX m_frontLeftDrive{1};
    WPI_CANCoder m_frontLeftSensor{2};
    
    WPI_TalonFX m_frontRightAngle{3};
    WPI_TalonFX m_frontRightDrive{4};
    WPI_CANCoder m_frontRightSensor{5};

    WPI_TalonFX m_rearLeftAngle{6};
    WPI_TalonFX m_rearLeftDrive{7};
    WPI_CANCoder m_rearLeftSensor{8};

    WPI_TalonFX m_rearRightAngle{9};
    WPI_TalonFX m_rearRightDrive{10};
    WPI_CANCoder m_rearRightSensor{11};

    rev::CANSparkMax arm_extend{20, rev::CANSparkMax::MotorType::kBrushless};
    WPI_TalonFX arm_angle{13};
    WPI_TalonFX m_intake{15};

    frc::DoubleSolenoid p_solenoidA{14, frc::PneumaticsModuleType::CTREPCM, 4, 5};
    frc::Compressor p_compressor{14, frc::PneumaticsModuleType::CTREPCM};

    frc::Timer timer;

    frc::Encoder arm_encoder{7, 8};
    frc::DigitalInput insideSwitch {0};
    frc::DigitalInput armSwitch {1};

    AHRS *ahrs;

    double conversionFactor = 4096.0/ 360.0;

    double frameLength = 25.75;
    double frameWidth = 25.75;
    double frameRatio = sqrt((frameLength * frameLength) + (frameWidth * frameWidth));

    //****************    NAVX-MXP (Gyro board)
    double yaw;

    double positionSpeed = .3;

    int resetTimer = 0;

    //Drivemotor PID loop values
    double drivekP = 0.001;
    double drivekD = 0.005;
    double drivekF = 1;
    double drivekI = 0;
  };
