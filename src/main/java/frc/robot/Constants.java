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
    public static final int kMotorCanId = 15;

    // ===================================================================================
    // UNIT CONVENTION
    // ===================================================================================
    // All position values are in DEGREES and velocity values are in DEGREES/SECOND.
    // The subsystem converts to rotations when configuring the motor.
    // To convert: 1 rotation = 360 degrees
    // ===================================================================================

    // ===================================================================================
    // GEAR RATIO CONFIGURATION
    // ===================================================================================
    // SensorToMechanismRatio = rotor rotations per mechanism rotation
    //
    // This value tells the motor controller how many rotor rotations equal one
    // mechanism rotation. The motor will report position/velocity in mechanism
    // rotations, not rotor rotations.
    //
    // Examples:
    //   - 1:1 direct drive: kGearRatio = 1.0
    //   - 12.8:1 gearbox:   kGearRatio = 12.8
    //   - 50:1 gearbox:     kGearRatio = 50.0
    //
    // IMPORTANT: When changing the gear ratio, you must also scale kP and kD:
    //   kP_new = kP_base * (old_ratio / new_ratio)
    //   kD_new = kD_base * (old_ratio / new_ratio)
    // The feedforward gains (kS, kV, kA) do NOT need to be scaled.
    // ===================================================================================
    public static final double kGearRatio = 17.77;  // 17.77:1 gearbox

    // ===================================================================================
    // MOTION MAGIC PARAMETERS (in degrees)
    // ===================================================================================
    // Motion Magic generates a smooth motion profile (trapezoidal or S-curve) to move
    // the motor from current position to target position.
    //
    // kCruiseVelocity: Maximum velocity during the move (degrees per second)
    // kAcceleration:   How fast to speed up/slow down (degrees per second squared)
    // kJerk:           How fast acceleration changes (degrees per second cubed)
    //                  Higher jerk = snappier response, lower jerk = smoother motion
    //
    // Phoenix 6 example values (for 12.8:1 gearbox, converted to degrees):
    //   CruiseVelocity = 1800 deg/s, Acceleration = 3600 deg/s², Jerk = 36000 deg/s³
    // ===================================================================================
    public static final double kCruiseVelocity = 180.0;   // degrees per second
    public static final double kAcceleration = 360.0;     // degrees per second squared
    public static final double kJerk = 3600.0;            // degrees per second cubed

    // ===================================================================================
    // PID AND FEEDFORWARD GAINS (in rotations - Phoenix 6 native units)
    // ===================================================================================
    // These gains control how the motor responds to position/velocity errors.
    // NOTE: PID gains remain in ROTATIONS because Phoenix 6 operates internally in
    // rotations. The subsystem converts position commands to rotations before sending.
    //
    // Feedforward gains (predict required voltage):
    //   kS: Static friction compensation (volts). Voltage needed to overcome friction
    //       at rest. Does NOT scale with gear ratio.
    //   kV: Velocity feedforward (volts per rotation/s). Voltage needed to maintain
    //       a given velocity. Does NOT scale with gear ratio.
    //   kA: Acceleration feedforward (volts per rotation/s²). Voltage needed to
    //       accelerate. Does NOT scale with gear ratio.
    //
    // Feedback gains (correct for errors):
    //   kP: Proportional gain (volts per rotation of error). Higher = stiffer response.
    //       MUST be scaled when gear ratio changes: kP_new = kP_base / gear_ratio
    //   kI: Integral gain (volts per rotation-second of accumulated error).
    //       Usually leave at 0 for Motion Magic.
    //   kD: Derivative gain (volts per rotation/s of error velocity). Adds damping.
    //       MUST be scaled when gear ratio changes: kD_new = kD_base / gear_ratio
    //
    // These values are scaled from Phoenix 6 example base values to 17.77:1 gearbox.
    // Base values: kS=0.25, kV=0.12, kA=0.01, kP=60, kI=0, kD=0.5
    // Scaled kP and kD by dividing by 17.77.
    // ===================================================================================
    public static final double kS = 0.5;     // Static friction (volts) - increased for gearbox friction
    public static final double kV = 0.12;    // Velocity feedforward (V per rotation/s)
    public static final double kA = 0.01;    // Acceleration feedforward (V per rotation/s²)
    public static final double kP = 12.0;    // Position proportional gain (tuned for 17.77:1)
    public static final double kI = 1.0 / 360.0;  // Integral gain (scaled for degrees)
    public static final double kD = 0.0281;  // Derivative gain (0.5 / 17.77)

    // ===================================================================================
    // SOFT LIMITS (in degrees)
    // ===================================================================================
    // Soft limits prevent the motor from moving beyond specified positions.
    // The motor will refuse to move past these limits even if commanded to.
    //
    // Use soft limits to protect mechanisms from over-travel when there are no
    // physical hard stops, or as a safety backup to hard stops.
    // ===================================================================================
    public static final boolean kSoftLimitsEnabled = true;
    public static final double kForwardSoftLimit = 100.0;   // +540 degrees
    public static final double kReverseSoftLimit = -100.0;  // -540 degrees

    // ===================================================================================
    // CURRENT LIMITS
    // ===================================================================================
    // Current limits protect the motor, mechanism, and battery from excessive current.
    // Supply current limit restricts current drawn from the battery.
    //
    // Typical values:
    //   - Light load / positioning: 20-30 A
    //   - Medium load: 40-60 A
    //   - Heavy load (drivetrain): 60-80 A
    // ===================================================================================
    public static final boolean kCurrentLimitEnabled = true;
    public static final double kSupplyCurrentLimit = 50.0;  // Amps

    // ===================================================================================
    // NEUTRAL MODE
    // ===================================================================================
    // Controls motor behavior when no power is applied (output = 0).
    //
    // Brake mode: Motor resists rotation, holds position. Good for arms, elevators,
    //             and mechanisms that need to hold position.
    // Coast mode: Motor spins freely. Good for flywheels or when you want the
    //             mechanism to coast to a stop naturally.
    // ===================================================================================
    public static final boolean kBrakeMode = true;
  }
}
