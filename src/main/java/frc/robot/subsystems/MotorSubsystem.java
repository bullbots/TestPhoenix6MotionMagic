// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private final TalonFX m_motor;
  private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);
  private double m_targetPosition = 0;

  public MotorSubsystem() {
    m_motor = new TalonFX(MotorConstants.kMotorCanId);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Configure gear ratio
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = MotorConstants.kGearRatio;

    // Configure Motion Magic (values are in rotations per second)
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = MotorConstants.kCruiseVelocity;  // rotations per second
    mm.MotionMagicAcceleration = MotorConstants.kAcceleration;      // rotations per second squared
    mm.MotionMagicJerk = MotorConstants.kJerk;                      // rotations per second cubed

    // Configure PID and feedforward gains
    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = MotorConstants.kS;
    slot0.kV = MotorConstants.kV;
    slot0.kA = MotorConstants.kA;
    slot0.kP = MotorConstants.kP;
    slot0.kI = MotorConstants.kI;
    slot0.kD = MotorConstants.kD;

    // Configure soft limits (values are in rotations)
    SoftwareLimitSwitchConfigs softLimits = cfg.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitEnable = MotorConstants.kSoftLimitsEnabled;
    softLimits.ReverseSoftLimitEnable = MotorConstants.kSoftLimitsEnabled;
    softLimits.ForwardSoftLimitThreshold = MotorConstants.kForwardSoftLimit;
    softLimits.ReverseSoftLimitThreshold = MotorConstants.kReverseSoftLimit;

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
   * @param positionRotations Target position in rotations
   */
  public void setPosition(double positionRotations) {
    m_targetPosition = positionRotations;
    m_motor.setControl(m_mmRequest.withPosition(positionRotations).withSlot(0));
  }

  /**
   * Resets the motor's position sensor to a specified value.
   * @param positionRotations Position in rotations to set as current position
   */
  public void resetPosition(double positionRotations) {
    m_motor.setPosition(positionRotations);
  }

  /**
   * @return Current position in rotations
   */
  public double getPosition() {
    return m_motor.getPosition().getValueAsDouble();
  }

  /**
   * @return Current velocity in rotations per second
   */
  public double getVelocity() {
    return m_motor.getVelocity().getValueAsDouble();
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
    SmartDashboard.putNumber("[Out] Motor Target (rot)", m_targetPosition);
    SmartDashboard.putNumber("[Out] Motor Position (rot)", getPosition());
    SmartDashboard.putNumber("[Out] Motor Velocity (rot/s)", getVelocity());
  }
}
