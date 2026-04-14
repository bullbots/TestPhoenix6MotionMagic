// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final MotorSubsystem m_motorSubsystem = new MotorSubsystem();

  public RobotContainer() {
    // Initialize SmartDashboard setpoint with default value
    SmartDashboard.putNumber("[In] Motor Setpoint (deg)", 0);

    configureBindings();
  }

  private void configureBindings() {
    // Motion Magic position control using SmartDashboard setpoint
    m_motorSubsystem.setDefaultCommand(
        m_motorSubsystem.setPositionCommand(
            () -> SmartDashboard.getNumber("[In] Motor Setpoint (deg)", 0)
        )
    );
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }

  public MotorSubsystem getMotorSubsystem() {
    return m_motorSubsystem;
  }
}
