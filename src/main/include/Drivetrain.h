// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/numbers>

#include "Systems/PigeonGyro.h"
#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() { m_gyro.Reset(); }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed = 
      4.4196_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      2 * wpi::numbers::pi};  // 1 rotation per second. It is unclear if this speed is accurate, it likely is not

 private:
 #if 0
  frc::Translation2d m_frontLeftLocation{+0.31369_m, +0.31369_m};
  frc::Translation2d m_frontRightLocation{+0.31369_m, -0.31369_m};
  frc::Translation2d m_backLeftLocation{-0.31369_m, +0.31369_m};
  frc::Translation2d m_backRightLocation{-0.31369_m, -0.31369_m};
 #else
  frc::Translation2d m_frontLeftLocation{+0.31369_m, -0.31369_m};
  frc::Translation2d m_frontRightLocation{+0.31369_m, +0.31369_m};
  frc::Translation2d m_backLeftLocation{-0.31369_m, -0.31369_m};
  frc::Translation2d m_backRightLocation{-0.31369_m, +0.31369_m};
 #endif

  //driveMotorChannel, turningMotorChannel, turningEncoderChannel, offset
  SwerveModule m_frontLeft{5, 1, 0, -146.60};
  SwerveModule m_frontRight{6, 2, 1, -74.53};
  SwerveModule m_backLeft{7, 3, 2, 138.25};
  SwerveModule m_backRight{8, 4, 3, 16.88};

  hb::pigeonGyro m_gyro{0};

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_gyro.GetRotation2d()};
};
