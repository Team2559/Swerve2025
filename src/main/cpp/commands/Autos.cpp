// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"
#include "commands/SwerveTrajectoryCommand.h"

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>

static auto trajectory =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ExampleTrajectory");

frc2::CommandPtr autos::FallbackAuto(DriveSubsystem &driveSubsystem) {
  return frc2::RunCommand(
             [&]() {
               driveSubsystem.Drive(-0.25_mps, 0.0_mps, 0.0_rad_per_s, true);
             },
             {&driveSubsystem})
      .Until([&]() -> bool {
        return units::math::abs(driveSubsystem.GetPose().X()) >= 1.0_m;
      })
      .BeforeStarting([]() { printf(">>>Running traditional auto\n"); });
}

frc2::CommandPtr autos::ExampleAuto(DriveSubsystem &driveSubsystem) {
  if (trajectory.has_value()) {
    return SwerveTrajectoryCommand(driveSubsystem, trajectory.value())
        .BeforeStarting([]() { printf(">>>Running trajectory auto\n"); });
  } else {
    // Default auto command to ensure the line is left even if the trajectory
    // fails to load.
    return FallbackAuto(driveSubsystem);
  }
}
