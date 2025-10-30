#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"

namespace autos {
  enum class AutoProgram {
    kFallback,
    kExample,
  };

  /**
   * Fallback auto to slowly drive one meter off line if loading the desired auto fails 
   */
  frc2::CommandPtr FallbackAuto(DriveSubsystem& driveSubsystem);

  /**
   * Example static factory for a path-based autonomous command.
   */
  frc2::CommandPtr ExampleAuto(DriveSubsystem& subsystem);
}  // namespace autos
