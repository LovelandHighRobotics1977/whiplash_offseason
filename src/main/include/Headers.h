#ifndef HEADERS_H
#define HEADERS_H

//c++
#include <iostream>
#include <math.h>

//frc kinematics
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

//units
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/dimensionless.h>

//motors and CAN devices
#include <ctre/Phoenix.h>
#include "rev/CANSparkMax.h"
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

//frc inputs
#include <frc/MathUtil.h> //for frc::ApplyDeadband
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/GenericHID.h>

//frc misc
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

//misc
#include <cameraserver/CameraServer.h>

//user defined
#include "Gyro.h"

#endif /* !HEADERS_H */