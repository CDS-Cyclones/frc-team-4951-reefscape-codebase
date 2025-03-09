// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.operation.manual.MoveIntakeWheelsManuallyCommand;
import frc.robot.commands.operation.manual.MovePivotManuallyCommand;
import frc.robot.subsystems.manipulator.Elevator;
import frc.robot.subsystems.manipulator.Pivot;
import frc.robot.subsystems.oi.OI;
import frc.robot.subsystems.manipulator.IntakeWheels;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOSpark;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final Drive swerve;
  private final Elevator m_elevator;
  private final Pivot m_pivot;
  private final IntakeWheels m_intakeWheels;

  private SwerveDriveSimulation swerveSimulation = null;

  // Autonomous command chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        swerve = new Drive(new GyroIOPigeon2(), new ModuleIOSpark(0), new ModuleIOSpark(1), new ModuleIOSpark(2), new ModuleIOSpark(3), (pose) -> {});
        break;

      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.swerveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveSimulation);
        
        // Sim robot, instantiate physics sim IO implementations
        swerve = new Drive(
          new GyroIOSim(swerveSimulation.getGyroSimulation()),
          new ModuleIOSim(swerveSimulation.getModules()[0]),
          new ModuleIOSim(swerveSimulation.getModules()[1]),
          new ModuleIOSim(swerveSimulation.getModules()[2]),
          new ModuleIOSim(swerveSimulation.getModules()[3]),
          swerveSimulation::setSimulationWorldPose
        );

        break;

      default:
        // Replayed robot, disable IO implementations
        swerve = new Drive(new GyroIO(){}, new ModuleIO(){}, new ModuleIO(){}, new ModuleIO(){}, new ModuleIO(){}, (pose) -> {});
        break;
    }

    // Set up PP auton routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    
    // Set up SysId routines
    autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(swerve));
    autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(swerve));
    autoChooser.addOption("Drive SysId (Quasistatic Forward)", swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Quasistatic Reverse)", swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (Dynamic Forward)", swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Dynamic Reverse)", swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_elevator = new Elevator();
    m_pivot = new Pivot();
    m_intakeWheels = new IntakeWheels();

    registerNamedCommands();
    configureBindings();
  }

  /**
   * Configure button bindings.
   */
  private void configureBindings() {
    // Default command, normal field-relative drive
    swerve.setDefaultCommand(DriveCommands.joystickDrive(
      swerve,
      () -> OI.m_driverController.getLeftY(),
      () -> OI.m_driverController.getLeftX(),
      () -> OI.m_driverController.getRightX())
    );

    // Lock to 0° when A button is held
    new JoystickButton(OI.m_driverController, Button.kA.value).whileTrue(DriveCommands.joystickDriveAtAngle(
      swerve,
      () -> -OI.m_driverController.getLeftY(),
      () -> -OI.m_driverController.getLeftX(),
      () -> new Rotation2d())
    );

    // Switch to X pattern when X button is pressed
    new JoystickButton(OI.m_driverController, Button.kX.value).onTrue(Commands.runOnce(swerve::stopWithX, swerve));

    // Reset gyro to 0° when B button is pressed
    final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
      ? () -> swerve.resetOdometry(swerveSimulation.getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
      : () -> swerve.resetOdometry(new Pose2d(swerve.getPose().getTranslation(), new Rotation2d())); // zero gyro
    new JoystickButton(OI.m_driverController, Button.kB.value).onTrue(Commands.runOnce(resetGyro, swerve).ignoringDisable(true));

    // Check if the robot is in test mode
    if (DriverStation.isTest()) {
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
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    swerve.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", swerveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
            "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
            "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
}
}
