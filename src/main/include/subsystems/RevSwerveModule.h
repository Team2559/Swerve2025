#pragma once

#include <rev/SparkFlex.h>

#include "SwerveModule.h"

using namespace rev::spark;

class RevSwerveModule: public SwerveModule {
 public:
  // Initializers
  RevSwerveModule(int driveCanID, int steerCanID, units::angle::turn_t offset);

  // Called every loop
  void Periodic() override;

  // Is the swerve module in a "healthy" state?
  bool GetStatus() const override;

  void SetSteerOffset(units::angle::turn_t offset) override;
  units::angle::turn_t GetSteerPosition() override;
  void SetSteerPosition(units::angle::turn_t position) override;
  void StopSteer() override;

  void SetDriveVelocity(units::velocity::meters_per_second_t velocity);
  void ResetDriveEncoder() override;
  void SetDrivePercent(double percent) override;

  const frc::SwerveModuleState GetState() override;
  const frc::SwerveModulePosition GetPosition() override;
  void SetDesiredState(const frc::SwerveModuleState &state) override;
  void Stop() override;

 private:
  SparkFlex driveMotor;
  SparkFlex steerMotor;

  SparkRelativeEncoder driveEncoder;
  SparkAbsoluteEncoder steerEncoder;
};