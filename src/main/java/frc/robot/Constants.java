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
    public static final int kMotorCanId = 0;

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
    public static final double kGearRatio = 1.0;  // 1:1 direct drive

    // ===================================================================================
    // MOTION MAGIC PARAMETERS
    // ===================================================================================
    // Motion Magic generates a smooth motion profile (trapezoidal or S-curve) to move
    // the motor from current position to target position. All values are in mechanism
    // rotations (after gear ratio is applied).
    //
    // kCruiseVelocity: Maximum velocity during the move (rotations per second)
    // kAcceleration:   How fast to speed up/slow down (rotations per second squared)
    // kJerk:           How fast acceleration changes (rotations per second cubed)
    //                  Higher jerk = snappier response, lower jerk = smoother motion
    //
    // Phoenix 6 example values (for 12.8:1 gearbox):
    //   CruiseVelocity = 5 rot/s, Acceleration = 10 rot/s², Jerk = 100 rot/s³
    // ===================================================================================
    public static final double kCruiseVelocity = 0.5;   // rotations per second
    public static final double kAcceleration = 1.0;     // rotations per second squared
    public static final double kJerk = 10.0;            // rotations per second cubed

    // ===================================================================================
    // PID AND FEEDFORWARD GAINS
    // ===================================================================================
    // These gains control how the motor responds to position/velocity errors.
    // All gains operate in mechanism rotations (after gear ratio is applied).
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
    // These values are scaled from Phoenix 6 example (12.8:1 gearbox) to 1:1 direct drive.
    // Base values: kS=0.25, kV=0.12, kA=0.01, kP=60, kI=0, kD=0.5
    // Scaled kP and kD by dividing by 12.8.
    // ===================================================================================
    public static final double kS = 0.25;    // Static friction (volts)
    public static final double kV = 0.12;    // Velocity feedforward (V per rotation/s)
    public static final double kA = 0.01;    // Acceleration feedforward (V per rotation/s²)
    public static final double kP = 4.7;     // Position proportional gain (60 / 12.8)
    public static final double kI = 0;       // Integral gain
    public static final double kD = 0.039;   // Derivative gain (0.5 / 12.8)

    // ===================================================================================
    // SOFT LIMITS
    // ===================================================================================
    // Soft limits prevent the motor from moving beyond specified positions.
    // The motor will refuse to move past these limits even if commanded to.
    // Values are in mechanism rotations (after gear ratio is applied).
    //
    // Use soft limits to protect mechanisms from over-travel when there are no
    // physical hard stops, or as a safety backup to hard stops.
    // ===================================================================================
    public static final boolean kSoftLimitsEnabled = false;
    public static final double kForwardSoftLimit = 1.5;   // +1.5 rotations
    public static final double kReverseSoftLimit = -1.5;  // -1.5 rotations

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
    public static final double kSupplyCurrentLimit = 30.0;  // Amps

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
