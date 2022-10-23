// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int turningEncoderChannel,
                           const double offset)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
      m_turningEncoder(turningEncoderChannel, offset), m_drivechannel(driveMotorChannel)
      {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveMotor.SetDistancePerPulse(2 * wpi::numbers::pi * kWheelRadius /
                                     kEncoderResolution);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::numbers::pi)
  // divided by the encoder resolution.
  m_turningEncoder.SetDistancePerPulse(2 * wpi::numbers::pi /
                                       kEncoderResolution);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      -units::radian_t(wpi::numbers::pi), units::radian_t(wpi::numbers::pi));

  
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveMotor.GetRate()},
          frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_turningEncoder.Get()));

  // Calculate the drive output from the drive PID controller.
    const auto driveOutput = m_drivePIDController.Calculate(
        m_driveMotor.GetRate(), state.speed.value());

    // Calculate the turning motor output from the turning PID controller.
    const auto turnOutput = m_turningPIDController.Calculate(
        units::radian_t(m_turningEncoder.Get()), state.angle.Radians());
      
      
    // Set the motor outputs.
    // if(fabs(state.speed.value()) < 0.001) {
    //   m_driveMotor.Set(0);
    //   m_turningMotor.Set(0);
    // } 
    // else {
    //   m_driveMotor.Set(driveOutput);
    //   m_turningMotor.Set(turnOutput);
    // }
      if (m_drivechannel == 6) printf("DriveOutput: %5.2f\n", driveOutput);
      m_driveMotor.Set(driveOutput/12);
      m_turningMotor.Set(turnOutput);
    // if modules are moving without input check Robot.cpp and increase deadbands
}
