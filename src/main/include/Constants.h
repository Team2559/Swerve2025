// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/constants.h>
#include <frc/geometry/Transform3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

  inline constexpr int kDriverControllerPort = 0;

}

namespace MotorConstants {
  // Speed constant for a Neo Vortex, in turns per second per volt
  constexpr units::unit_t<units::compound_unit<units::turns_per_second, units::inverse<units::volt>>> kVNeoVortex = 6784_rpm / 12.0_V;
}

namespace DriveConstants {
  // Drivebase geometry: distance between centers of right and left wheels on
  // robot; distance between centers of front and back wheels on robot.
  inline constexpr units::meter_t kWheelbaseWidth = 0.7_m;
  inline constexpr units::meter_t kWheelbaseLength = 0.7_m;

  // Zero positions for the steer of the swerve modules
  inline constexpr units::degree_t kFrontLeftSteerOffset  = 17.2_deg;
  inline constexpr units::degree_t kFrontRightSteerOffset = 89.2_deg;
  inline constexpr units::degree_t kRearLeftSteerOffset   = 325.6_deg;
  inline constexpr units::degree_t kRearRightSteerOffset  = 244.4_deg;

  // SDS Mk3 Standard (or Fast) Gear Ratio: 8.16:1 (or 6.86:1);
  // Nominal Wheel Diameter (4"): =0.1016m;
  // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
  // 8.16 / 0.3192 => ~25.57.

  // SDS Mk4 L1, L2, L3, L4 Gear Ratio: 8.14:1, 6.75:1, 6.12:1, 5.14:1
  // Nominal Wheel Diameter (4"): =0.1016m;
  // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
  // 6.75 / 0.3192 => ~21.15.

  // SDS Mk4n L1, L2, L3 Gear Ratio: 7.13:1, 5.90:1, 5.36:1
  // Nominal Wheel Diameter (4"): =0.1016m;
  // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
  // 0.3192 / 5.90 => ~0.05401.

  inline constexpr double kDriveGearRatio = 50.0/16.0 * 17.0/27.0 * 45.0/15.0; // approx 5.90

  // This should be empirically determined!  This is just an initial guess.
  // This is used for both distance and velocity control. If this is off, it
  // will throw off kMaxDriveSpeed and kMaxTurnRate, as well as drive values.
  inline constexpr units::unit_t<units::compound_unit<units::meter, units::inverse<units::turn>>> kDriveDistancePerRotation = 4.00_in * units::constants::pi / units::turn_t{kDriveGearRatio};

  // SDS Mk3 Standard (or Fast) Max Free Speed: 12.1 (or 14.4) feet/second;
  // Review your motor and swerve module configuration for nominal free speed
  // This is an upper bound, for various reasons. It needs to be empirically
  // measured. Half of theoretical free speed is a reasonable starting value
  // (since something in the ballpark is needed here in order to to drive).
  constexpr units::meters_per_second_t kMaxDriveSpeed = 6895_rpm * kDriveDistancePerRotation;
  constexpr double kSlowDrivePercent = 0.80;

  // This is used for rotating the robot in place, about it's center.  This
  // may need to be empirically adjusted, but check kDriveMetersPerRotation
  // before making any adjustment here.
  const units::meter_t kDriveMetersPerSteerCircle = M_PI * units::math::sqrt(units::math::pow<2>(kWheelbaseLength) + units::math::pow<2>(kWheelbaseWidth));

  // This is the maximum rotational speed -- not of a swerve module, but of
  // the entire robot.  This is a function of the maximum drive speed and the
  // geometry of the robot.  This will occur when the robot spins in place,
  // around the center of a circle which passes through all the drive modules
  // (if there is no single such circle, things are analogous).  If the drive
  // modules are turned to be tangential to this circle and run at maximum,
  // the robot is rotating as fast as possible.  This can be derived from
  // kMaxDriveSpeed and the geometry and does not have to be directly
  // measured.  It is a good idea to check this value empirically though.

  // So the maximum rotational velocity (spinning in place) is kMaxDriveSpeed
  // / kDriveMetersPerSteerCircle * 360 degrees.  This should not need to
  // be empirically adjusted (but check).
  const units::degrees_per_second_t kMaxTurnRate =
      kMaxDriveSpeed / kDriveMetersPerSteerCircle * 360.0_deg;

  // CAN ID assignments.
  constexpr int kFrontLeftDriveMotorCanID = 1;
  constexpr int kFrontLeftSteerMotorCanID = 2;
  constexpr int kRearLeftDriveMotorCanID = 3;
  constexpr int kRearLeftSteerMotorCanID = 4;
  constexpr int kRearRightDriveMotorCanID = 5;
  constexpr int kRearRightSteerMotorCanID = 6;
  constexpr int kFrontRightDriveMotorCanID = 7;
  constexpr int kFrontRightSteerMotorCanID = 8;
  
  // These can flip because of gearing.
  constexpr bool kDriveMotorInverted = false;
  constexpr bool kSteerMotorInverted = true;
  constexpr bool kSteerSensorInverted = false;

  // Closed loop feedback parameters for module drive speed
  namespace DrivePID {
    constexpr double kP = 0.2;
    constexpr double kI = 0.0;
    constexpr double kD = 0.002;
    constexpr double kV = (1.0 / MotorConstants::kVNeoVortex / kDriveDistancePerRotation).value();
  }

  // Steer encoder units are scaled for more responsive PID feedback
  constexpr double kSteerFeedbackScale = 10.0;
  constexpr double kInvSteerFeedbackScale = 1 / kSteerFeedbackScale;

  // Closed loop feedback parameters for module steer position
  namespace SteerPID {
    constexpr double kP = 0.8;
    constexpr double kI = 0.0;
    constexpr double kD = 0.03;
    constexpr double kFF = 0.0;
  }

  // TODO: Closed loop feedback for chassis speed and orientation
}

namespace VisionConstants {
    // Camera network name
    constexpr char kName[] = "OV9281";
    // AprilTag field data; FMA events should all use the welded field, but off-season events may use the AndyMark field instead.
    const frc::AprilTagFieldLayout kAprilTags = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded);
    // Camera focal point position and orientation relative to the robot origin
    constexpr frc::Transform3d kRobotToCam = frc::Transform3d(frc::Translation3d(0.5_m, 0_m, 0.5_m),
                    frc::Rotation3d(0_rad, 0_rad, 0_rad));
  }