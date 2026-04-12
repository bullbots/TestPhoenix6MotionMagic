// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorConstants {
    public static final int kMotorCanId = 1;

    // ===================================================================================
    // GEAR RATIO CONFIGURATION
    // ===================================================================================
    // SensorToMechanismRatio = (rotor rotations per mechanism rotation) * (unit conversion)
    //
    // Example: For a 17.77:1 gearbox measuring in degrees:
    //   kGearRatio = 17.77 * 360.0 = 6397.2
    //
    // For testing with 1:1 ratio in degrees:
    //   kGearRatio = 1.0 * 360.0 = 360.0
    //
    // After setting kGearRatio, you must also scale the gains (see PID section below).
    // ===================================================================================
    private static final double kMechanicalGearRatio = 1.0;  // Rotor rotations per mechanism rotation
    private static final double kUnitsPerRotation = 360.0;   // 360 for degrees, 1 for rotations
    public static final double kGearRatio = kMechanicalGearRatio * kUnitsPerRotation;

    // ===================================================================================
    // MOTION MAGIC PARAMETERS
    // ===================================================================================
    // These values are in mechanism units (degrees if kUnitsPerRotation = 360).
    // Base values (for 1 rotation = 1 unit): 5 units/s, 10 units/s², 100 units/s³
    // Scaled by kUnitsPerRotation for degrees.
    // ===================================================================================
    public static final double kCruiseVelocity = 5 * kUnitsPerRotation;   // degrees per second
    public static final double kAcceleration = 10 * kUnitsPerRotation;    // degrees per second squared
    public static final double kJerk = 100 * kUnitsPerRotation;           // degrees per second cubed

    // ===================================================================================
    // PID AND FEEDFORWARD GAINS
    // ===================================================================================
    // Gains are applied in mechanism units. When you change units (e.g., rotations to
    // degrees), position/velocity errors become numerically larger for the same physical
    // error, so gains must be scaled down proportionally.
    //
    // Base values (tuned for rotations, i.e., kUnitsPerRotation = 1):
    //   kS = 0.25 V (static friction - not scaled, it's a voltage offset)
    //   kV = 0.12 V per rotation/s
    //   kA = 0.01 V per rotation/s²
    //   kP = 60 V per rotation of error
    //   kI = 0
    //   kD = 0.5 V per rotation/s of error velocity
    //
    // To convert for different units, divide velocity/position gains by kUnitsPerRotation:
    //   kV_new = kV_base / kUnitsPerRotation
    //   kP_new = kP_base / kUnitsPerRotation
    //   etc.
    //
    // Example: kP = 60 for rotations. For degrees: kP = 60 / 360 = 0.167
    //   A 72° error (0.2 rotations) gives: 72 * 0.167 = 12V (same as 0.2 * 60 = 12V)
    // ===================================================================================
    public static final double kS = 0.25;                      // Static friction (volts) - not scaled
    public static final double kV = 0.12 / kUnitsPerRotation;  // Velocity feedforward
    public static final double kA = 0.01 / kUnitsPerRotation;  // Acceleration feedforward
    public static final double kP = 60.0 / kUnitsPerRotation;  // Position proportional gain
    public static final double kI = 0;                         // Integral gain
    public static final double kD = 0.5 / kUnitsPerRotation;   // Derivative gain

    // ===================================================================================
    // SOFT LIMITS
    // ===================================================================================
    // Soft limits prevent the motor from moving beyond specified positions.
    // Values are in mechanism units (degrees if kUnitsPerRotation = 360).
    // The motor will not accept setpoints or move beyond these limits.
    // ===================================================================================
    public static final boolean kSoftLimitsEnabled = true;
    public static final double kForwardSoftLimit = 540.0;   // +540 degrees (360 + 180)
    public static final double kReverseSoftLimit = -540.0;  // -540 degrees (360 + 180)

    // ===================================================================================
    // CURRENT LIMITS
    // ===================================================================================
    // Current limits protect the motor and mechanism from excessive current draw.
    // Supply current limit restricts current drawn from the battery.
    // ===================================================================================
    public static final boolean kCurrentLimitEnabled = true;
    public static final double kSupplyCurrentLimit = 30.0;  // Amps

    // ===================================================================================
    // NEUTRAL MODE
    // ===================================================================================
    // Brake mode holds position when no power is applied.
    // Coast mode allows free spinning when no power is applied.
    // ===================================================================================
    public static final boolean kBrakeMode = true;
  }
}
