// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverJoystickConstants;
import frc.robot.Constants.VisionConstants.PosesRelToAprilTags;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  // Swerve subsystems
  private final SwerveSubsystem m_Swerve;

  // Vision subsystem
  private final VisionSubsystem m_VisionSubsystem;

  // Subsystem that periodically estimates pose based on odometry and vision readings
  @SuppressWarnings("unused")
  private final PoseEstimatorSubsystem m_PoseEstimator;

  // Driver controller
  private final CommandXboxController m_DriverController;

  // Autonomous command chooser
  private final SendableChooser<Command> autoChooser;

  // Whether the robot should drive in field relative mode or robot relative mode
  private boolean fieldOriented = true;


  public RobotContainer() {
    // Initialize
    m_Swerve = new SwerveSubsystem();
    m_VisionSubsystem = new VisionSubsystem();
    m_PoseEstimator = new PoseEstimatorSubsystem(m_Swerve, m_VisionSubsystem);
    m_DriverController = new CommandXboxController(DriverJoystickConstants.kDriverControllerPort);

    // Set default driving command
    m_Swerve.setDefaultCommand(
      new TeleopDriveCommand(
        m_Swerve,
        m_DriverController::getLeftY,
        m_DriverController::getLeftX,
        m_DriverController::getRightX,
        () -> fieldOriented,
        m_DriverController.getHID()::getBButton
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
    m_DriverController.x().onTrue((Commands.runOnce(m_Swerve::zeroGyro)));
    m_DriverController.y().whileTrue(new ChaseTagCommand(m_VisionSubsystem, m_Swerve, m_Swerve::getPose, PosesRelToAprilTags.SAMPLE_POSE));
    m_DriverController.a().onTrue(Commands.runOnce(() -> fieldOriented = !fieldOriented));
    
    // SysId Routines for Swerve
    // m_DriverController.x().onTrue(m_Swerve.sysIdDriveMotorCommand());
    // m_DriverController.b().onTrue(m_Swerve.sysIdAngleMotorCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
