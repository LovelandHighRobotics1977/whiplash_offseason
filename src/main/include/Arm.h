// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Headers.h>

/**
 * Represents a grabber arm using the Climber in a box
*/
class Arm {
    public:
        Arm();

        /**
         * Extension of the Arm "climber"
         * @note Usually uses xbox controller bumpers.
         * @param in Moves the arm in.
         * @param out Moves the arm out.
        */
        void Extension(int in, int out);

        /**
         * Angle of the mechanism arm
         * @note Usually uses xbox controller joysticks.
         * @param up Moves the arm up.
         * @param down Moves the arm down.
        */
        void Angle(float up, float down);

        /**
         * Intake control for the roller intake on the end of the arm
         * @note Usually uses xbox controller buttons
         * @param in Moves the rollers to pick up an object.
         * @param out Moves the rollers to place an object.
        */
        void Intake(int in, int out);

        /**
         * Automatic placement of the arm for scoring or stowing.  
         * @note Usually uses xbox controller buttons
         * @param angle Angle of the arm to approach where 0 is direct down.  
         * @param extending Is the am extending or retracting?
         * @param enabled Enable arm automovement?
        */
        int AutoPosition(int angle, bool raising, bool lowering);
    private:
        rev::CANSparkMax m_armExtend{20, rev::CANSparkMax::MotorType::kBrushless};
		WPI_TalonFX m_armAngle{13};
		WPI_TalonFX m_armIntake{15};

        frc::Encoder m_armEncoder{7, 8};
		frc::DigitalInput m_angleSwitch {0};
		frc::DigitalInput m_extensionSwitch {1};

        double positionSpeed = .3;
};