// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <studica/AHRS.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/estimator/SwerveDrivePoseEstimator3d.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <networktables/GenericEntry.h>
#include <choreo/Choreo.h>

#include "Constants.h"
#include "PIDTuner.h"
#include "SwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void TestInit();
  void TestExit();

  /** 
   * Zero out the drive distance
   */
  void ResetDrive();

  /**
   *  Reset forward for the driver to be the way the robot is currently facing
   */
  void ResetFieldOrientation(bool inverted = false);

  /**
   * Move at the requested set of speeds measured relative to either the robot or the field
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative)
  {
    Drive(xSpeed, ySpeed, rot, fieldRelative, 0.0_m, 0.0_m);
  }

  /**
   * Move at the requested set of speeds measured relative to either the robot or the field, with an arbitrary center of rotation
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::meter_t x_center, units::meter_t y_center);

  /**
   * Move towards the choreo trajectory sample position (with PID feedback)
   */
  void FollowTrajectory(const choreo::SwerveSample& sample);

  /**
   * Move the module steering to be aligned for the requested direction of travel
   */
  void SteerTo(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
               bool fieldRelative)
  {
    SteerTo(xSpeed, ySpeed, rot, fieldRelative, 0.0_m, 0.0_m);
  }
  
  /**
   * Move the module steering to be aligned for the requested direction of travel, with an arbitrary center of rotation
   */
  void SteerTo(units::meters_per_second_t xSpeed,
              units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
              bool fieldRelative, units::meter_t x_center, units::meter_t y_center);

  /**
   * Stop all drivetrain movement
   */
  void Stop();

  /**
   * Reset the robot's pose to the provided pose
   */
  void ResetPose(frc::Pose3d pose);

  /**
   * Gets the robot's current pose (position + orientation) 
   */
  frc::Pose3d GetPose();

  /**
   * Incorporate a vision pose measurement into the robots cumulative pose estimation
   */
  void UpdateVisionPose(frc::Pose3d measurement, units::millisecond_t timestamp);

  /**
   * Get the current steer angle and wheel positions for all modules for odometry
   */
  const std::array<frc::SwerveModulePosition, 4> GetModulePositions();

  /**
   * Sets the desired steer angle and wheel velocity for all modules
   */
  void SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates, bool steerOnly=false);

  // Front: +x, Rear: -x; Left: +y, Right -y. Zero heading is to the front
  // and +rotation is counter-clockwise. This is all standard, although it
  // means the robot's front is along the x-axis, which is often pointed to
  // the right, as things are occasionally drawn.
  /**
   * Drive kinematics for converting robot velocities into module velocities and vice-versa
   */
  frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d(+DriveConstants::kWheelbaseLength / 2, +DriveConstants::kWheelbaseWidth / 2),
      frc::Translation2d(+DriveConstants::kWheelbaseLength / 2, -DriveConstants::kWheelbaseWidth / 2),
      frc::Translation2d(-DriveConstants::kWheelbaseLength / 2, +DriveConstants::kWheelbaseWidth / 2),
      frc::Translation2d(-DriveConstants::kWheelbaseLength / 2, -DriveConstants::kWheelbaseWidth / 2)};

  /**
   * Field drawing for dashboard debug information
   */
  frc::Field2d field;

 private:
  // The four swerve modules.
  std::unique_ptr<SwerveModule> frontLeftModule;
  std::unique_ptr<SwerveModule> frontRightModule;
  std::unique_ptr<SwerveModule> rearLeftModule;
  std::unique_ptr<SwerveModule> rearRightModule;

  // The navX gyro sensor.
  std::unique_ptr<studica::AHRS> m_ahrs;

  // Pose estimator combines odometry with vision readings to yield an accurate robot pose; 4 specifies the number of modules.
  std::unique_ptr<frc::SwerveDrivePoseEstimator3d<4>> m_poseEstimator;

  // Tuners to adjust PID values live from the dashboard; greatly increases the ease of tuning
  PIDTuner m_driveTuner;
  PIDTuner m_steerTuner;

  // PID controllers for autonomous path following
  frc::PIDController m_xController;
  frc::PIDController m_yController;
  frc::PIDController m_rController;


  // Dashboard logging entries
  // -----------------------------------

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
