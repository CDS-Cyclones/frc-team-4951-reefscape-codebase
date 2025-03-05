// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.operation.manual.MoveElevatorManuallyCommand;
import frc.robot.commands.operation.manual.MoveIntakeWheelsManuallyCommand;
import frc.robot.commands.operation.manual.MovePivotManuallyCommand;
import frc.robot.commands.operation.pid.ElevatorGoToCommand;
import frc.robot.commands.operation.pid.PivotGoToCommand;
import frc.robot.subsystems.manipulator.Elevator;
import frc.robot.subsystems.manipulator.Pivot;
import frc.robot.subsystems.manipulator.IntakeWheels;
import frc.robot.subsystems.oi.OI;
import frc.robot.subsystems.swerve.Swerve;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class RobotContainer {
  private final Swerve m_swerve;
  private final Elevator m_elevator;
  private final Pivot m_pivot;
  private final IntakeWheels m_intakeWheels;

  // Autonomous command chooser
  private final SendableChooser<Command> autoChooser;

  // Whether the robot should drive in field relative mode or robot relative mode
  private boolean fieldOriented = true;

  public RobotContainer() throws IOException, ParseException {
    // Initialize subsystems
    m_swerve = new Swerve();
    m_elevator = new Elevator();
    m_pivot = new Pivot();
    m_intakeWheels = new IntakeWheels();

    // Set default driving commands
    m_swerve.setDefaultCommand(
      new TeleopDriveCommand(
        m_swerve,
        OI.m_driverController::getLeftY,
        OI.m_driverController::getLeftX,
        OI.m_driverController::getRightX,
        () -> fieldOriented,
        OI.m_driverController::getBButton
      )
    );

    m_elevator.setDefaultCommand(
      new ElevatorGoToCommand(
        m_elevator,
          ManipulatorSubsystemsPositions.getCurrentElevatorPositionSupplier()
      )
    );

    m_pivot.setDefaultCommand(
      new PivotGoToCommand(
        m_pivot,
        ManipulatorSubsystemsPositions.getCurrentPivotPositionSupplier()
      )
    );

    // Auto chooser for selecting pathplanner trajectories
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  /**
   * Configure button bindings.
   */
  private void configureBindings() {
    new JoystickButton(OI.m_driverController, Button.kX.value).onTrue(new RunCommand(m_swerve::zeroHeading, m_swerve));
    new JoystickButton(OI.m_driverController, Button.kY.value).whileTrue(new RunCommand(m_swerve::setX, m_swerve));
    new JoystickButton(OI.m_driverController, Button.kB.value).onTrue(new RunCommand(() -> fieldOriented = !fieldOriented));;

    // Check if the robot is in test mode
    if (DriverStation.isTest()) {
      new JoystickButton(OI.m_mainpulatorControllerManualBackup, Button.kY.value).whileTrue(new MoveElevatorManuallyCommand(m_elevator, m_pivot, true));
      new JoystickButton(OI.m_mainpulatorControllerManualBackup, Button.kA.value).whileTrue(new MoveElevatorManuallyCommand(m_elevator, m_pivot, false));

      new JoystickButton(OI.m_mainpulatorControllerManualBackup, Button.kB.value).whileTrue(new MovePivotManuallyCommand(m_pivot, true));
      new JoystickButton(OI.m_mainpulatorControllerManualBackup, Button.kX.value).whileTrue(new MovePivotManuallyCommand(m_pivot, false));

      new JoystickButton(OI.m_mainpulatorControllerManualBackup, Button.kLeftBumper.value).whileTrue(new MoveIntakeWheelsManuallyCommand(m_intakeWheels, 1));
      new JoystickButton(OI.m_mainpulatorControllerManualBackup, Button.kRightBumper.value).whileTrue(new MoveIntakeWheelsManuallyCommand(m_intakeWheels, -1));
    }
  }

  /**
   * Register named commands for PathPlanner autos.
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("elevatorDown", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.DOWN)));
    NamedCommands.registerCommand("elevatorL1", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.L1)));
    NamedCommands.registerCommand("elevatorL2", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.L2)));
    NamedCommands.registerCommand("elevatorL3", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.L3)));
    NamedCommands.registerCommand("elevatorL4", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.L4)));
    NamedCommands.registerCommand("elevatorBarge", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.BARGE)));

    NamedCommands.registerCommand("pivotIn", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentPivotPosition(ManipulatorSubsystemsPositions.PivotPosition.IN)));
    NamedCommands.registerCommand("pivotOut", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentPivotPosition(ManipulatorSubsystemsPositions.PivotPosition.OUT)));
  }

  /**
   * Returns the selected autonomous command.
   *
   * @return the selected autonomous command
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
