// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriverJoystickConstants;
import frc.robot.commands.AimAndGetInRangeCommand;
import frc.robot.commands.AimWhileDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionCamera;

public class RobotContainer {
  private final SwerveSubsystem m_Swerve;

  private final VisionCamera m_Camera;

  private final CommandXboxController m_DriverController;

  private final SendableChooser<Command> autoChooser;



  public RobotContainer() {
    // Initialize
    m_Swerve = new SwerveSubsystem();
    m_Camera = new VisionCamera("cds_cam", new Transform3d());
    m_DriverController = new CommandXboxController(DriverJoystickConstants.kDriverControllerPort);

    // Set default driving command
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    m_Swerve.setDefaultCommand(
      m_Swerve.driveCommand(
        () -> MathUtil.applyDeadband(m_DriverController.getLeftY() * -1, DriverJoystickConstants.kLeftXDeadband),
        () -> MathUtil.applyDeadband(m_DriverController.getLeftX() * -1, DriverJoystickConstants.kLeftYDeadband),
        () -> MathUtil.applyDeadband(m_DriverController.getRightX() * -1, DriverJoystickConstants.kRightXDeadband) * DriverJoystickConstants.kTurnMultiplier,
        true,
        false
      )
    );


    // Register Named Commands for PP autons
    // ex. NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());

    // Auto chooser for selection PP trajectories
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    m_DriverController.a().onTrue((Commands.runOnce(m_Swerve::zeroGyro)));
    // m_DriverController.x().whileTrue(new AimWhileDriveCommand(m_Swerve, m_Camera, m_DriverController).repeatedly());
    // m_DriverController.y().whileTrue(new AimAndGetInRangeCommand(m_Swerve, m_Camera, m_DriverController).repeatedly());
  

    // SysId Routines for Swerve
    // m_DriverController.x().onTrue(m_Swerve.sysIdDriveMotorCommand());
    // m_DriverController.b().onTrue(m_Swerve.sysIdAngleMotorCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}