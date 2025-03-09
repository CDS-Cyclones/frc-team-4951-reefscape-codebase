// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.DesiredFieldPose.DrivePose;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.operation.manual.MoveIntakeWheelsManuallyCommand;
import frc.robot.commands.operation.manual.MovePivotManuallyCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.manipulator.Elevator;
import frc.robot.subsystems.manipulator.Pivot;
import frc.robot.subsystems.oi.OI;
import frc.robot.subsystems.manipulator.IntakeWheels;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final Drive drive;
  private final Vision vision;
  private final Elevator m_elevator;
  private final Pivot m_pivot;
  private final IntakeWheels m_intakeWheels;

  private SwerveDriveSimulation driveSimulation = null;

  // Autonomous command chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new GyroIOPigeon2(), new ModuleIOSpark(0), new ModuleIOSpark(1), new ModuleIOSpark(2), new ModuleIOSpark(3), (pose) -> {});
        vision = new Vision(drive, new VisionIOLimelight(VisionConstants.cameraName, drive::getRotation));
        break;

      case SIM:
        // create a maple-sim swerve drive simulation instance
        driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
          new GyroIOSim(driveSimulation.getGyroSimulation()),
          new ModuleIOSim(driveSimulation.getModules()[0]),
          new ModuleIOSim(driveSimulation.getModules()[1]),
          new ModuleIOSim(driveSimulation.getModules()[2]),
          new ModuleIOSim(driveSimulation.getModules()[3]),
          driveSimulation::setSimulationWorldPose
        );
        vision = new Vision(drive, new VisionIOPhotonVisionSim(VisionConstants.cameraNameSim, VisionConstants.botToCamTransformSim, driveSimulation::getSimulatedDriveTrainPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new GyroIO(){}, new ModuleIO(){}, new ModuleIO(){}, new ModuleIO(){}, new ModuleIO(){}, (pose) -> {});
        vision = new Vision(drive, new VisionIO() {});
        break;
    }

    // Set up PP auton routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    
    // Set up SysId routines
    autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    drive.setDefaultCommand(DriveCommands.joystickDrive(
      drive,
      () -> -OI.m_driverController.getLeftY(),
      () -> -OI.m_driverController.getLeftX(),
      () -> -OI.m_driverController.getRightX())
    );

    // Lock to 0° when A button is held
    new JoystickButton(OI.m_driverController, Button.kA.value).whileTrue(DriveCommands.joystickDriveAtAngle(
      drive,
      () -> -OI.m_driverController.getLeftY(),
      () -> -OI.m_driverController.getLeftX(),
      DesiredFieldPose::getDriveRotation2d
    ));

    // Switch to X pattern when X button is pressed
    new JoystickButton(OI.m_driverController, Button.kX.value).onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
      ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
      : () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    new JoystickButton(OI.m_driverController, Button.kB.value).onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // POSES
    new JoystickButton(OI.m_operatorBoard, 7).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.A)));
    new JoystickButton(OI.m_operatorBoard, 8).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.B)));
    new JoystickButton(OI.m_operatorBoard, 9).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.C)));
    new JoystickButton(OI.m_operatorBoard, 10).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.D)));
    new JoystickButton(OI.m_operatorBoard, 11).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.E)));
    new JoystickButton(OI.m_operatorBoard, 12).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.F)));
    new JoystickButton(OI.m_operatorBoard, 1).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.G)));
    new JoystickButton(OI.m_operatorBoard, 2).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.H)));
    new JoystickButton(OI.m_operatorBoard, 3).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.I)));
    new JoystickButton(OI.m_operatorBoard, 4).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.J)));
    new JoystickButton(OI.m_operatorBoard, 5).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.K)));
    new JoystickButton(OI.m_operatorBoard, 6).onTrue(Commands.runOnce(() -> DesiredFieldPose.setDrivePose(DrivePose.L)));
    

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
    // NamedCommands.registerCommand("elevatorDown", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.DOWN)));
    // NamedCommands.registerCommand("elevatorL1", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.L1)));
    // NamedCommands.registerCommand("elevatorL2", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.L2)));
    // NamedCommands.registerCommand("elevatorL3", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.L3)));
    // NamedCommands.registerCommand("elevatorL4", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.L4)));
    // NamedCommands.registerCommand("elevatorBarge", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentElevatorPosition(ManipulatorSubsystemsPositions.ElevatorPosition.BARGE)));

    // NamedCommands.registerCommand("pivotIn", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentPivotPosition(ManipulatorSubsystemsPositions.PivotPosition.IN)));
    // NamedCommands.registerCommand("pivotOut", new RunCommand(() -> ManipulatorSubsystemsPositions.setCurrentPivotPosition(ManipulatorSubsystemsPositions.PivotPosition.OUT)));
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

    drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
            "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
            "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
}
}
