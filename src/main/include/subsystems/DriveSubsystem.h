// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <studica/AHRS.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

#include "SwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  // Zero drive distance
  void ResetDrive();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void ResetFieldOrientation();

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative)
  {
    Drive(xSpeed, ySpeed, rot, fieldRelative, 0.0_m, 0.0_m);
  }
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::meter_t x_center, units::meter_t y_center);


  void SteerTo(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
               bool fieldRelative)
  {
    SteerTo(xSpeed, ySpeed, rot, fieldRelative, 0.0_m, 0.0_m);
  }
  void SteerTo(units::meters_per_second_t xSpeed,
              units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
              bool fieldRelative, units::meter_t x_center, units::meter_t y_center);

  void Stop();
  
  frc::Pose2d GetPose();

  const std::array<frc::SwerveModulePosition, 4> GetModulePositions();
  void SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates, bool steerOnly=false);

  // Front: +x, Rear: -x; Left: +y, Right -y. Zero heading is to the front
  // and +rotation is counter-clockwise. This is all standard, although it
  // means the robot's front is along the x-axis, which is often pointed to
  // the right, as things are commonly drawn. Rotate the page by 90 degrees.
  frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d(+DriveConstants::kWheelbaseLength / 2, +DriveConstants::kWheelbaseWidth / 2),
      frc::Translation2d(+DriveConstants::kWheelbaseLength / 2, -DriveConstants::kWheelbaseWidth / 2),
      frc::Translation2d(-DriveConstants::kWheelbaseLength / 2, +DriveConstants::kWheelbaseWidth / 2),
      frc::Translation2d(-DriveConstants::kWheelbaseLength / 2, -DriveConstants::kWheelbaseWidth / 2)};

 private:
  // The four swerve modules.
  std::unique_ptr<SwerveModule> frontLeftModule;
  std::unique_ptr<SwerveModule> frontRightModule;
  std::unique_ptr<SwerveModule> rearLeftModule;
  std::unique_ptr<SwerveModule> rearRightModule;

  // The navX gyro sensor.
  std::unique_ptr<studica::AHRS> m_ahrs;

  // Pose estimator combines odometry with vision readings to yield an accurate robot pose; 4 specifies the number of modules.
  std::unique_ptr<frc::SwerveDrivePoseEstimator<4>> m_poseEstimator;

  nt::GenericEntry *nt_xPosition;
  nt::GenericEntry *nt_xSetpoint;
  nt::GenericEntry *nt_xOutput;

  nt::GenericEntry *nt_yPosition;
  nt::GenericEntry *nt_ySetpoint;
  nt::GenericEntry *nt_yOutput;

  nt::GenericEntry *nt_rPosition;
  nt::GenericEntry *nt_rSetpoint;
  nt::GenericEntry *nt_rOutput;
};
