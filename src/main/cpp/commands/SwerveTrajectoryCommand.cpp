// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SwerveTrajectoryCommand.h"

#include <frc/DriverStation.h>

using namespace choreo;

SwerveTrajectoryCommand::SwerveTrajectoryCommand(
    DriveSubsystem &subsystem, Trajectory<SwerveSample> &trajectory)
    : m_driveSubsytem{subsystem}, m_trajectory{trajectory}, m_timer{} {
  AddRequirements(&subsystem);
}

void SwerveTrajectoryCommand::Initialize() {
  // Check which alliance we are on to determine what side of the field to run
  // the trajectory on
  m_invertForRed = frc::DriverStation::GetAlliance().value_or(
                       frc::DriverStation::kBlue) == frc::DriverStation::kRed;
  // Reset the timer that tracks our progress through the trajectory
  m_timer.Restart();
  // Reset the robot pose to the starting pose of the trajectory
  auto initialPose = m_trajectory.GetInitialPose(m_invertForRed);
  if (initialPose.has_value()) {
    m_driveSubsytem.ResetPose(frc::Pose3d(initialPose.value()));
  }
  // Update the field display on the dashboard to show the expected trajectory
  // path
  m_driveSubsytem.field.GetObject("traj")->SetPoses(
      (m_invertForRed ? m_trajectory.Flipped() : m_trajectory).GetPoses());
}

void SwerveTrajectoryCommand::Execute() {
  // Sample a pose from the trajectory, then attempt to drive to that pose
  std::optional<SwerveSample> sample =
      m_trajectory.SampleAt(m_timer.Get(), m_invertForRed);
  if (sample.has_value()) {
    m_driveSubsytem.FollowTrajectory(sample.value());
  }
}

void SwerveTrajectoryCommand::End(bool wasCanceled) {
  // Stop the drivetrain and clear the trajectory from the dashboard field
  m_driveSubsytem.Stop();
  m_driveSubsytem.field.GetObject("traj")->SetPoses({});
}

bool SwerveTrajectoryCommand::IsFinished() {
  // End if the expected trajectory duration has elapsed
  return m_timer.Get() > m_trajectory.GetTotalTime();
}
