// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

import java.util.function.DoubleSupplier;

public class MotorSubsystem extends SubsystemBase {
  private static final double kDegreesPerRotation = 360.0;

  private final TalonFX m_motor;
  private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);
  private double m_targetPositionDegrees = 0;

  public MotorSubsystem() {
    m_motor = new TalonFX(MotorConstants.kMotorCanId, new CANBus("Indiana"));

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Configure gear ratio
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = MotorConstants.kGearRatio;

    // Configure Motion Magic (convert from degrees to rotations)
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = MotorConstants.kCruiseVelocity / kDegreesPerRotation;
    mm.MotionMagicAcceleration = MotorConstants.kAcceleration / kDegreesPerRotation;
    mm.MotionMagicJerk = MotorConstants.kJerk / kDegreesPerRotation;

    // Configure PID and feedforward gains
    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = MotorConstants.kS;
    slot0.kV = MotorConstants.kV;
    slot0.kA = MotorConstants.kA;
    slot0.kP = MotorConstants.kP;
    slot0.kI = MotorConstants.kI;
    slot0.kD = MotorConstants.kD;

    // Configure soft limits (convert from degrees to rotations)
    SoftwareLimitSwitchConfigs softLimits = cfg.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitEnable = MotorConstants.kSoftLimitsEnabled;
    softLimits.ReverseSoftLimitEnable = MotorConstants.kSoftLimitsEnabled;
    softLimits.ForwardSoftLimitThreshold = MotorConstants.kForwardSoftLimit / kDegreesPerRotation;
    softLimits.ReverseSoftLimitThreshold = MotorConstants.kReverseSoftLimit / kDegreesPerRotation;

    // Configure neutral mode (brake or coast)
    MotorOutputConfigs motorOutput = cfg.MotorOutput;
    motorOutput.NeutralMode = MotorConstants.kBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Configure current limits
    CurrentLimitsConfigs currentLimits = cfg.CurrentLimits;
    currentLimits.SupplyCurrentLimitEnable = MotorConstants.kCurrentLimitEnabled;
    currentLimits.SupplyCurrentLimit = MotorConstants.kSupplyCurrentLimit;

    // Apply configuration with retry
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_motor.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure motor. Error: " + status.toString());

    }

    // Reset position to 0 on startup
    m_motor.setPosition(0);
  }

  /**
   * Sets the target position using Motion Magic.
   * @param positionDegrees Target position in degrees
   */
  public void setPosition(double positionDegrees) {
    m_targetPositionDegrees = positionDegrees;
    double positionRotations = positionDegrees / kDegreesPerRotation;
    m_motor.setControl(m_mmRequest.withPosition(positionRotations).withSlot(0));
  }

  /**
   * Resets the motor's position sensor to a specified value.
   * @param positionDegrees Position in degrees to set as current position
   */
  public void resetPosition(double positionDegrees) {
    double positionRotations = positionDegrees / kDegreesPerRotation;
    m_motor.setPosition(positionRotations);
  }

  /**
   * @return Current position in degrees
   */
  public double getPosition() {
    return m_motor.getPosition().getValueAsDouble() * kDegreesPerRotation;
  }

  /**
   * @return Current velocity in degrees per second
   */
  public double getVelocity() {
    return m_motor.getVelocity().getValueAsDouble() * kDegreesPerRotation;
  }

  public TalonFX getMotor() {
    return m_motor;
  }

  public Command setPositionCommand(DoubleSupplier positionSupplier) {
    return run(() -> setPosition(positionSupplier.getAsDouble()));
  }

  public Command resetPositionCommand(double positionRotations) {
    return runOnce(() -> resetPosition(positionRotations));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("[Out] Motor Target (deg)", m_targetPositionDegrees);
    SmartDashboard.putNumber("[Out] Motor Position (deg)", getPosition());
    SmartDashboard.putNumber("[Out] Motor Velocity (deg/s)", getVelocity());

    // Check soft limit faults
    boolean forwardLimitHit = m_motor.getFault_ForwardSoftLimit().getValue();
    boolean reverseLimitHit = m_motor.getFault_ReverseSoftLimit().getValue();
    SmartDashboard.putBoolean("[Out] Forward Limit Hit", forwardLimitHit);
    SmartDashboard.putBoolean("[Out] Reverse Limit Hit", reverseLimitHit);

    if (forwardLimitHit) {
      System.out.println("WARNING: Forward soft limit reached at " + getPosition() + " degrees");
    }
    if (reverseLimitHit) {
      System.out.println("WARNING: Reverse soft limit reached at " + getPosition() + " degrees");
    }
  }
}
