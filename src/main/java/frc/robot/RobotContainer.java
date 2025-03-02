// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverJoystickConstants;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.Constants.OperatorBoardConstants;
import frc.robot.Constants.OperatorJoystickConstants;
import frc.robot.Constants.VisionConstants.PoseRelToAprilTag;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.operation.ElevatorGoToCommand;
import frc.robot.commands.operation.IntakeInCommand;
import frc.robot.commands.operation.IntakeOutCommand;
import frc.robot.commands.operation.MoveArmManuallyCommand;
import frc.robot.commands.operation.MoveElevatorManuallyCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OperatorBoard;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.USBCameraSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  // Swerve subsystems
  private final SwerveSubsystem m_Swerve;
  
  // Elevator subsystem
  private final ElevatorSubsystem m_Elevator;

  private final ArmSubsystem m_Arm;

  private final IntakeSubsystem m_Intake;

  // Vision subsystem
  private final VisionSubsystem m_Vision;

  // Subsystem that periodically estimates pose based on odometry and vision readings
  @SuppressWarnings("unused")
  private final PoseEstimatorSubsystem m_PoseEstimator;

  // Xbox controllers
  private final CommandXboxController m_DriverController, m_OperatorController;

  // Custom operator board
  @SuppressWarnings("unused")
  private final OperatorBoard m_OperatorBoard;

  // Autonomous command chooser
  private final SendableChooser<Command> autoChooser;

  private final USBCameraSubsystem cam;

  // Whether the robot should drive in field relative mode or robot relative mode
  private boolean fieldOriented = true;


  public RobotContainer() {
    // Initialize
    m_Swerve = new SwerveSubsystem();
    m_Elevator = new ElevatorSubsystem();
    m_Arm = new ArmSubsystem();
    m_Intake = new IntakeSubsystem();
    m_Vision = new VisionSubsystem();
    m_PoseEstimator = new PoseEstimatorSubsystem(m_Swerve, m_Vision);
    m_DriverController = new CommandXboxController(DriverJoystickConstants.kDriverControllerPort);
    m_OperatorController = new CommandXboxController(OperatorJoystickConstants.kOperatorControllerPort);
    m_OperatorBoard = new OperatorBoard(OperatorBoardConstants.kOperatorBoardPort);
    cam = new USBCameraSubsystem("camera", 0);

    // Set default driving command
    m_Swerve.setDefaultCommand(
      new TeleopDriveCommand(
        m_Swerve,
        m_Vision,
        m_DriverController::getLeftY,
        m_DriverController::getLeftX,
        m_DriverController::getRightX,
        () -> fieldOriented,
        m_DriverController.getHID()::getBButton
      )
    );

    // Set default elevator command
    m_Elevator.setDefaultCommand(new MoveElevatorManuallyCommand(m_Elevator, m_OperatorController::getRightY));

    // Set defauilt command for arm
    m_Arm.setDefaultCommand(new MoveArmManuallyCommand(m_Arm, m_OperatorController::getLeftY));

    // Register Named Commands for PP autons
    // ex. NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());

    // Auto chooser for selection PP trajectories
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    m_DriverController.x().onTrue((Commands.runOnce(m_Swerve::zeroGyro)));
    // m_DriverController.y().whileTrue(new ChaseTagCommand(m_Vision, m_Swerve, m_Swerve::getPose, PoseRelToAprilTag.SAMPLE_POSE));
    // m_DriverController.rightBumper().onTrue(Commands.runOnce(() -> fieldOriented = !fieldOriented));
    
    // check if inb test mode
    if(DriverStation.isTest()) {
      m_DriverController.b().onTrue(Commands.runOnce(m_Swerve::resetOdometry));

      // Try pivoting around wheels
      m_DriverController.povUpLeft().whileTrue(Commands.run(() -> m_Swerve.drive(.5 * m_Swerve.getMaximumDriveVelocity(), .5 * m_Swerve.getMaximumDriveVelocity(), 0, new Translation2d(12.5, 12.5))));
      m_DriverController.povUpRight().whileTrue(Commands.run(() -> m_Swerve.drive(.5 * m_Swerve.getMaximumDriveVelocity(), -.5 * m_Swerve.getMaximumDriveVelocity(), 0, new Translation2d(12.5, -12.5))));
      m_DriverController.povDownLeft().whileTrue(Commands.run(() -> m_Swerve.drive(-.5 * m_Swerve.getMaximumDriveVelocity(), .5 * m_Swerve.getMaximumDriveVelocity(), 0, new Translation2d(-12.5, 12.5))));
      m_DriverController.povDownRight().whileTrue(Commands.run(() -> m_Swerve.drive(-.5 * m_Swerve.getMaximumDriveVelocity(), -.5 * m_Swerve.getMaximumDriveVelocity(), 0, new Translation2d(-12.5, -12.5))));
    }

    m_DriverController.rightBumper().whileTrue(new IntakeOutCommand(m_Intake));
    m_DriverController.leftBumper().whileTrue(new IntakeInCommand(m_Intake));

    // SysId Routines for Swerve
    // m_DriverController.x().onTrue(m_Swerve.sysIdDriveMotorCommand());
    // m_DriverController.b().onTrue(m_Swerve.sysIdAngleMotorCommand());

    // m_OperatorController.a().whileTrue(new ElevatorGoToCommand(m_Elevator, ElevatorPosition.L1));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
