// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverJoystickConstants;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.Constants.OperatorBoardConstants;
import frc.robot.Constants.OperatorJoystickConstants;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.operation.ElevatorGoToCommand;
import frc.robot.commands.operation.manual.IntakeAlgaManuallyCommand;
import frc.robot.commands.operation.manual.OuttakeCoralManuallyCommand;
import frc.robot.commands.operation.manual.MoveArmManuallyCommand;
import frc.robot.commands.operation.manual.MoveElevatorManuallyCommand;
import frc.robot.commands.operation.manual.OuttakeAlgaManuallyCommand;
import frc.robot.commands.operation.pid.ArmInCommand;
import frc.robot.commands.operation.pid.ArmOutCommand;
import frc.robot.commands.operation.timed.IntakeAlgaTimedCommand;
import frc.robot.commands.operation.timed.OuttakeAlgaTimedCommand;
import frc.robot.commands.operation.timed.OuttakeCoralTimedCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OperatorBoard;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.USBCameraSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final SwerveSubsystem m_swerve;
  private final ElevatorSubsystem m_Elevator;
  private final ArmSubsystem m_Arm;
  private final IntakeSubsystem m_Intake;
  private final VisionSubsystem m_Vision;
  private final PoseEstimatorSubsystem m_PoseEstimator;
  private final XboxController m_driverController, m_operatorController;
  private final OperatorBoard m_OperatorBoard;
  private final USBCameraSubsystem cam;

  // Autonomous command chooser
  private final SendableChooser<Command> autoChooser;

  // Whether the robot should drive in field relative mode or robot relative mode
  private boolean fieldOriented = true;

  public RobotContainer() {
    // Initialize suubsystems
    m_swerve = new SwerveSubsystem();
    m_Elevator = new ElevatorSubsystem();
    m_Arm = new ArmSubsystem();
    m_Intake = new IntakeSubsystem();
    m_Vision = new VisionSubsystem();
    m_PoseEstimator = new PoseEstimatorSubsystem(null, m_Vision);
    m_driverController = new XboxController(DriverJoystickConstants.kDriverControllerPort);
    m_operatorController = new XboxController(OperatorJoystickConstants.kOperatorControllerPort);
    m_OperatorBoard = new OperatorBoard(OperatorBoardConstants.kOperatorBoardPort);
    cam = new USBCameraSubsystem("camera", 0);

    // Set default commands
    m_swerve.setDefaultCommand(
      new TeleopDriveCommand(
        m_swerve,
        m_Vision,
        m_driverController::getLeftY,
        m_driverController::getLeftX,
        m_driverController::getRightX,
        () -> fieldOriented,
        m_driverController::getBButton
      )
    );
    // m_Elevator.setDefaultCommand(new MoveElevatorManuallyCommand(m_Elevator, m_Arm, m_OperatorController::getLeftY));
    // m_Arm.setDefaultCommand(new MoveArmManuallyCommand(m_Arm, m_OperatorController::getRightY));

    // Register commands for pathplanner autons
    NamedCommands.registerCommand("armOut", new ArmOutCommand(m_Arm));
    NamedCommands.registerCommand("armIn", new ArmInCommand(m_Arm));
    NamedCommands.registerCommand("elevatorL1", new ElevatorGoToCommand(m_Elevator, m_Arm, ElevatorPosition.L1));
    NamedCommands.registerCommand("elevatorL2", new ElevatorGoToCommand(m_Elevator, m_Arm, ElevatorPosition.L2));
    NamedCommands.registerCommand("elevatorL3", new ElevatorGoToCommand(m_Elevator, m_Arm, ElevatorPosition.L3));
    NamedCommands.registerCommand("elevatorL4", new ElevatorGoToCommand(m_Elevator, m_Arm, ElevatorPosition.L4));
    NamedCommands.registerCommand("elevatorBarge", new ElevatorGoToCommand(m_Elevator, m_Arm, ElevatorPosition.BARGE));
    NamedCommands.registerCommand("intakeAlga", new IntakeAlgaTimedCommand(m_Intake));
    NamedCommands.registerCommand("outtakeAlga", new OuttakeAlgaTimedCommand(m_Intake));
    NamedCommands.registerCommand("intakeCoral", new IntakeAlgaTimedCommand(m_Intake));
    NamedCommands.registerCommand("outtakeCoral", new OuttakeCoralTimedCommand(m_Intake));

    // Auto chooser for selecting pathplanner trajectories
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_driverController, Button.kX.value).onTrue(new RunCommand(() -> m_swerve.zeroHeading(), m_swerve));
    new JoystickButton(m_driverController, Button.kY.value).whileTrue(new RunCommand(() -> m_swerve.setX(), m_swerve));
    new JoystickButton(m_driverController, Button.kB.value).onTrue(new RunCommand(() -> fieldOriented = !fieldOriented));
    
    // m_DriverController.y().whileTrue(new ChaseTagCommand(m_Vision, m_Swerve, m_Swerve::getPose, PoseRelToAprilTag.SAMPLE_POSE));
    // m_DriverController.rightBumper().onTrue(Commands.runOnce(() -> fieldOriented = !fieldOriented));

    // m_OperatorController.leftBumper().whileTrue(new IntakeAlgaManuallyCommand(m_Intake));
    // m_DriverController.rightBumper().whileTrue(new OuttakeAlgaManuallyCommand(m_Intake));
    // m_OperatorController.x().whileTrue(new OuttakeCoralManuallyCommand(m_Intake));
    // m_OperatorController.a().whileTrue(new ElevatorGoToCommand(m_Elevator, ElevatorPosition.L1));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
