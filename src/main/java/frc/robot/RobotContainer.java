// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverJoystickConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem m_Swerve = new SwerveSubsystem();

  private final CommandXboxController m_DriverController = new CommandXboxController(DriverJoystickConstants.kDriverControllerPort);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = m_Swerve.driveCommand(
    () -> MathUtil.applyDeadband(m_DriverController.getLeftX() * -1, DriverJoystickConstants.kLeftXDeadband),
    () -> MathUtil.applyDeadband(m_DriverController.getLeftY() * -1, DriverJoystickConstants.kLeftYDeadband),
    () -> MathUtil.applyDeadband(m_DriverController.getRightX() * -1, DriverJoystickConstants.kRightXDeadband) * DriverJoystickConstants.kTurnMultiplier,
    false,
    false
  );

  public RobotContainer() {
    configureBindings();

    m_Swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  private void configureBindings() {
    m_DriverController.a().onTrue((Commands.runOnce(m_Swerve::zeroGyro)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}