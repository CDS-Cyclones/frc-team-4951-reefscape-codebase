// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drive.AutoDriveToPoseCommand;
import frc.robot.commands.drive.DriveCharacterizationCommands;
import frc.robot.commands.drive.JoystickDriveCommand;
import frc.robot.commands.drive.VisionAssistedDriveToPoseCommand;
import frc.robot.commands.elevator.ElevatorToPositionCommand;
import frc.robot.commands.elevator.HoldElevatorPositionCommand;
import frc.robot.commands.elevator.ManualElevatorCommand;
import frc.robot.commands.intake.IntakeCoralCommand;
import frc.robot.commands.intake.ManualIntakeCommand;
import frc.robot.commands.intake.ScoreCoralCommand;
import frc.robot.commands.pivot.HoldPivotPositionCommand;
import frc.robot.commands.pivot.ManualPivotCommand;
import frc.robot.commands.pivot.PivotToPositionCommand;
import frc.robot.mutables.MutableElevatorPosition;
import frc.robot.mutables.MutableFieldPose;
import frc.robot.mutables.MutablePivotPosition;
import frc.robot.mutables.MutableElevatorPosition.ElevatorPosition;
import frc.robot.mutables.MutableFieldPose.FieldPose;
import frc.robot.mutables.MutablePivotPosition.PivotPosition;
import frc.robot.sequences.ScoreSequence;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.oi.OI;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.utils.TunableValues;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Pivot pivot;
  private final Intake intake;

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

    // Instantiate manipulator subsystems
    elevator = new Elevator();
    pivot = new Pivot();
    intake = new Intake();

    registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    setupSysIdRoutines();
    setMutableDefaults();
    configureBindings();

    TunableValues.setTuningMode(true);  // TODO turn off for competition
  }

  public void setMutableDefaults() {
    MutableFieldPose.setMutableFieldPose(FieldPose.I);
    MutableElevatorPosition.setMutableElevatorPosition(ElevatorPosition.L4);
    MutablePivotPosition.setMutablePivotPosition(PivotPosition.L4);
  }

  /**
   * Configure button bindings.
   */
  private void configureBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(new JoystickDriveCommand(
      drive,
      () -> -OI.m_driverController.getLeftY(),
      () -> -OI.m_driverController.getLeftX(),
      () -> -OI.m_driverController.getRightX())
    );

    // Default command for elevator, hold position
    elevator.setDefaultCommand(new HoldElevatorPositionCommand(elevator));

    // Default command for pivot, hold position
    pivot.setDefaultCommand(new HoldPivotPositionCommand(pivot));

    // Locks robot's orientation to desired angle and vision aims whenever desired tag is detected
    new JoystickButton(OI.m_driverController, Button.kLeftBumper.value).whileTrue(new VisionAssistedDriveToPoseCommand(
      drive,
      vision,
      () -> -OI.m_driverController.getLeftY(),
      () -> -OI.m_driverController.getLeftX(),
      MutableFieldPose::getMutableFieldPose
    ));

    // Run scoring sequence whenever right bumper is clicked
    new JoystickButton(OI.m_driverController, Button.kRightBumper.value).whileTrue(new ScoreSequence(
      elevator,
      pivot,
      intake
    ));

    // Switch to X pattern when X button is pressed
    new JoystickButton(OI.m_driverController, Button.kX.value).onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
      ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
      : () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    new JoystickButton(OI.m_driverController, Button.kB.value).onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // Operator board bindings
    new JoystickButton(OI.m_operatorBoard, 1).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.A)));
    new JoystickButton(OI.m_operatorBoard, 2).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.B)));
    new JoystickButton(OI.m_operatorBoard, 3).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.C)));
    new JoystickButton(OI.m_operatorBoard, 4).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.D)));
    new JoystickButton(OI.m_operatorBoard, 5).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.E)));
    new JoystickButton(OI.m_operatorBoard, 6).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.F)));
    new JoystickButton(OI.m_operatorBoard, 7).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.G)));
    new JoystickButton(OI.m_operatorBoard, 8).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.H)));
    new JoystickButton(OI.m_operatorBoard, 9).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.I)));
    new JoystickButton(OI.m_operatorBoard, 10).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.J)));
    new JoystickButton(OI.m_operatorBoard, 11).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.K)));
    new JoystickButton(OI.m_operatorBoard, 12).onTrue(Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.L)));
    new JoystickButton(OI.m_operatorBoard, 13).onTrue(
      Commands.sequence(
          Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.INTAKE_LEFT)),
          new IntakeCoralCommand(intake)
      )
    );
    new JoystickButton(OI.m_operatorBoard, 14).onTrue(
      Commands.sequence(
          Commands.runOnce(() -> MutableFieldPose.setMutableFieldPose(FieldPose.INTAKE_RIGHT)),
          new IntakeCoralCommand(intake)
      )
    );
    new JoystickButton(OI.m_operatorBoard, 18).onTrue(Commands.runOnce(() -> {
      MutableElevatorPosition.setMutableElevatorPosition(ElevatorPosition.BARGE); 
      MutablePivotPosition.setMutablePivotPosition(PivotPosition.BARGE);
    }));
    new JoystickButton(OI.m_operatorBoard, 19).onTrue(Commands.runOnce(() -> {
      MutableElevatorPosition.setMutableElevatorPosition(ElevatorPosition.L4); 
      MutablePivotPosition.setMutablePivotPosition(PivotPosition.L4);
    }));
    new JoystickButton(OI.m_operatorBoard, 20).onTrue(Commands.runOnce(() -> {
      MutableElevatorPosition.setMutableElevatorPosition(ElevatorPosition.L3); 
      MutablePivotPosition.setMutablePivotPosition(PivotPosition.L3);
    }));
    new JoystickButton(OI.m_operatorBoard, 21).onTrue(Commands.runOnce(() -> {
      MutableElevatorPosition.setMutableElevatorPosition(ElevatorPosition.L2); 
      MutablePivotPosition.setMutablePivotPosition(PivotPosition.L2);
    }));

    // Bindings for manual manipulator controller
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kY.value)
      .whileTrue(new ManualElevatorCommand(elevator, pivot, () -> 0.2));
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kA.value)
      .whileTrue(new ManualElevatorCommand(elevator, pivot, () -> -0.2));
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kB.value)
      .whileTrue(new ManualPivotCommand(pivot, () -> 0.2));
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kX.value)
      .whileTrue(new ManualPivotCommand(pivot, () -> -0.2));
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kRightBumper.value)
      .whileTrue(new ManualIntakeCommand(intake, () -> 0.5 * (OI.m_mainpulatorControllerManual.getRawButton(Button.kStart.value) ? 2 : 1)));
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kLeftBumper.value)
      .whileTrue(new ManualIntakeCommand(intake, () -> -0.5 * (OI.m_mainpulatorControllerManual.getRawButton(Button.kStart.value) ? 2 : 1)));

    new JoystickButton(OI.m_manipulatorController, Button.kA.value)
      .whileTrue(new ElevatorToPositionCommand(elevator, pivot, MutableElevatorPosition::getTunableElevatorPositionAsDouble));
    new JoystickButton(OI.m_manipulatorController, Button.kB.value)
      .whileTrue(new PivotToPositionCommand(pivot, MutablePivotPosition::getTunablePivotPositionAsDouble));
  }

  /**
   * Setup SysId routines for the robot.
   */
  private void setupSysIdRoutines() {
    autoChooser.addOption("Drive Wheel Radius Characterization", DriveCharacterizationCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption("Drive Simple FF Characterization", DriveCharacterizationCommands.feedforwardCharacterization(drive));
    autoChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Elevator SysId (Quasistatic Forward)", elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Elevator SysId (Quasistatic Reverse)", elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Elevator SysId (Dynamic Forward)", elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Elevator SysId (Dynamic Reverse)", elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Pivot SysId (Quasistatic Forward)", pivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Pivot SysId (Quasistatic Reverse)", pivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Pivot SysId (Dynamic Forward)", pivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Pivot SysId (Dynamic Reverse)", pivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Register named commands for PathPlanner autos.
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("elevator__down", new ElevatorToPositionCommand(elevator, pivot, () -> ElevatorPosition.DOWN.getAsDouble()));
    NamedCommands.registerCommand("elevator__l1", new ElevatorToPositionCommand(elevator, pivot, () -> ElevatorPosition.L1.getAsDouble()));
    NamedCommands.registerCommand("elevator__l2", new ElevatorToPositionCommand(elevator, pivot, () -> ElevatorPosition.L2.getAsDouble()));
    NamedCommands.registerCommand("elevator__l3", new ElevatorToPositionCommand(elevator, pivot, () -> ElevatorPosition.L3.getAsDouble()));
    NamedCommands.registerCommand("elevator__l4", new ElevatorToPositionCommand(elevator, pivot, () -> ElevatorPosition.L4.getAsDouble()));
    NamedCommands.registerCommand("elevator__barge", new ElevatorToPositionCommand(elevator, pivot, () -> ElevatorPosition.BARGE.getAsDouble()));

    NamedCommands.registerCommand("pivot__intake_ready", new PivotToPositionCommand(pivot, () -> PivotPosition.INTAKE_READY.getAsDouble()));
    NamedCommands.registerCommand("pivot__elevator_clear", new PivotToPositionCommand(pivot, () -> PivotPosition.ELEVATOR_CLEAR.getAsDouble()));
    NamedCommands.registerCommand("pivot__l1", new PivotToPositionCommand(pivot, () -> PivotPosition.L1.getAsDouble()));
    NamedCommands.registerCommand("pivot__l2", new PivotToPositionCommand(pivot, () -> PivotPosition.L2.getAsDouble()));
    NamedCommands.registerCommand("pivot__l3", new PivotToPositionCommand(pivot, () -> PivotPosition.L3.getAsDouble()));
    NamedCommands.registerCommand("pivot__l4", new PivotToPositionCommand(pivot, () -> PivotPosition.L4.getAsDouble()));
    NamedCommands.registerCommand("pivot__barge", new PivotToPositionCommand(pivot, () -> PivotPosition.BARGE.getAsDouble()));

    NamedCommands.registerCommand("align__tagA", new AutoDriveToPoseCommand(drive, vision, FieldPose.A));
    NamedCommands.registerCommand("align__tagB", new AutoDriveToPoseCommand(drive, vision, FieldPose.B));
    NamedCommands.registerCommand("align__tagC", new AutoDriveToPoseCommand(drive, vision, FieldPose.C));
    NamedCommands.registerCommand("align__tagD", new AutoDriveToPoseCommand(drive, vision, FieldPose.D));
    NamedCommands.registerCommand("align__tagE", new AutoDriveToPoseCommand(drive, vision, FieldPose.E));
    NamedCommands.registerCommand("align__tagF", new AutoDriveToPoseCommand(drive, vision, FieldPose.F));
    NamedCommands.registerCommand("align__tagG", new AutoDriveToPoseCommand(drive, vision, FieldPose.G));
    NamedCommands.registerCommand("align__tagH", new AutoDriveToPoseCommand(drive, vision, FieldPose.H));
    NamedCommands.registerCommand("align__tagI", new AutoDriveToPoseCommand(drive, vision, FieldPose.I));
    NamedCommands.registerCommand("align__tagJ", new AutoDriveToPoseCommand(drive, vision, FieldPose.J));
    NamedCommands.registerCommand("align__tagK", new AutoDriveToPoseCommand(drive, vision, FieldPose.K));
    NamedCommands.registerCommand("align__tagL", new AutoDriveToPoseCommand(drive, vision, FieldPose.L));

    NamedCommands.registerCommand("intake_coral", new IntakeCoralCommand(intake));
    NamedCommands.registerCommand("score_coral", new ScoreCoralCommand(intake));
  }

  /**
   * Returns the selected autonomous command.
   *
   * @return the selected autonomous command
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Resets the MapleSim simulation field.
   */
  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(2,2))); 
  }

  /**
   * Updates the MapleSim simulation.
   */
  public void updateSimulation() {  
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
