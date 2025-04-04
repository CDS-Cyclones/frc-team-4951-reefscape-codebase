// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.RobotStateConstants.ReefHeight;
import frc.robot.Constants.RobotStateConstants.RobotAction;
import frc.robot.commands.drive.DriveCharacterizationCommands;
import frc.robot.commands.drive.JoystickDriveCommand;
import frc.robot.commands.intake.CoralGuardianCommand;
import frc.robot.commands.intake.IntakeActionCommand;
import frc.robot.commands.intake.IntakeCoralCommand;
import frc.robot.commands.intake.ManualIntakeCommand;
import frc.robot.Constants.RobotStateConstants.ElevatorPosition;
import frc.robot.Constants.RobotStateConstants.FieldPose;
import frc.robot.Constants.RobotStateConstants.IntakeAction;
import frc.robot.Constants.RobotStateConstants.PivotPosition;
import frc.robot.sequences.PositionManipulator;
import frc.robot.sequences.RetractManipulator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Candle;
import frc.robot.subsystems.oi.OI;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.utils.TunableValues;
import static frc.robot.Constants.DriveConstants.fineTuneSpeedMultiplier;
import static frc.robot.Constants.RobotStateConstants;

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
  private final Candle candle;

  private SwerveDriveSimulation driveSimulation = null;

  private byte sysIdRoutineId = 1;

  // Autonomous command chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  private final PathPlannerPath rightAuton;

  public RobotContainer() {
    try {
      rightAuton = PathPlannerPath.fromChoreoTrajectory("Test1");
    } catch(Exception e) {
      throw new RuntimeException("Failed to load RightAuton path", e);
    }

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new GyroIOPigeon2(), new ModuleIOSpark(0), new ModuleIOSpark(1), new ModuleIOSpark(2), new ModuleIOSpark(3), (pose) -> {});
        vision = new Vision(drive, new VisionIOLimelight(VisionConstants.limelightFrontName, drive::getRotation));
        elevator = new Elevator();
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
        elevator = new ElevatorSim();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new GyroIO(){}, new ModuleIO(){}, new ModuleIO(){}, new ModuleIO(){}, new ModuleIO(){}, (pose) -> {});
        vision = new Vision(drive, new VisionIO() {});
        elevator = new Elevator();
        break;
    }

    // Instantiate other subsystems
    pivot = new Pivot();
    intake = new Intake();
    candle = new Candle();

    registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("Test", AutoBuilder.followPath(rightAuton));
    setupSysIdRoutines();
    configureBindings();
    configureAutonEvents();

    TunableValues.setTuningMode(false);

    // Set up default states
    RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    RobotStateManager.setReefHeight(ReefHeight.L4);
    RobotStateManager.setAlignForAlgaePickup(false);
    RobotStateManager.setCoralScoringPose(FieldPose.E);
    RobotStateManager.setIntakeOccupied(false);

    // Call these to make sure the tunable values are loaded
    RobotStateConstants.tunableElevatorPosition.get();
    RobotStateConstants.tunablePivotPosition.get();
    RobotStateConstants.tunableIntakeSpeed.get();
    RobotStateConstants.tunableIntakeTime.get();
  }

  /**
   * Configure button bindings.
   */
  private void configureBindings() {
    ////////////////////////////////// DEFAULT COMMANDS //////////////////////////////////
    // Default command for drive, normal field-relative driving
    drive.setDefaultCommand(new JoystickDriveCommand(
      drive,
      () -> -OI.m_driverController.getLeftY() * (OI.m_driverController.getRawButton(Button.kB.value) ? fineTuneSpeedMultiplier : 1),
      () -> -OI.m_driverController.getLeftX() * (OI.m_driverController.getRawButton(Button.kB.value) ? fineTuneSpeedMultiplier : 1),
      () -> -OI.m_driverController.getRightX() * (OI.m_driverController.getRawButton(Button.kB.value) ? fineTuneSpeedMultiplier : 1),
      () -> !OI.m_operatorBoard.getRawButton(25)
    ));

    elevator.setDefaultCommand(Commands.run(() -> elevator.setVoltage(elevator.calculateFeedforward(0)), elevator));

    pivot.setDefaultCommand(
      new ConditionalCommand(
        Commands.run(() -> pivot.setVoltage(pivot.calculateFeedforward(0)), pivot),
        Commands.none(),
        () -> intake.isIntakeContainsCoral()
      )
    );

    // By default, the intake will guard the coral from falling out
    // if the intake contains coral else it will do nothing
    intake.setDefaultCommand(
      new ConditionalCommand(
        new CoralGuardianCommand(intake),
        new ConditionalCommand(
          Commands.run(() -> intake.setSpeed(-0.1), intake),
          Commands.none(),
          RobotStateManager::isAlignForAlgaePickup
        ),
        () -> intake.isIntakeContainsCoral()
      )
    );
    ////////////////////////////////////////////////////////////////////////////////////////


    ////////////////////////////////// DRIVER CONTROLLER //////////////////////////////////
    // Locks robot's orientation to desired angle and vision aims whenever desired tag is detected
    // new JoystickButton(OI.m_driverController, Button.kLeftBumper.value)
    //   .whileTrue(new VisionAssistedDriveToPoseCommand(
    //     drive,
    //     vision,
    //     elevator,
    //     candle,
    //     () -> -OI.m_driverController.getLeftY() * (OI.m_driverController.getRawButton(Button.kB.value) ? fineTuneSpeedMultiplier : 1),
    //     () -> -OI.m_driverController.getLeftX() * (OI.m_driverController.getRawButton(Button.kB.value) ? fineTuneSpeedMultiplier : 1),
    //     RobotStateManager::getDesiredFieldPose
    //   ));

    // // Scoring sequence
    // final Command scoringSequence = Commands.sequence(
    //   new PositionManipulator(
    //     elevator,
    //     pivot,
    //     RobotStateManager::getDesiredElevatorPosition,
    //     RobotStateManager::getDesiredPivotPosition
    //   ),
    //   Commands.waitUntil(() -> OI.m_driverController.getRightTriggerAxis() > 0.5 && RobotStateManager.getDesiredIntakeAction() != IntakeAction.OCCUPIED),
    //   new IntakeActionCommand(intake, candle, RobotStateManager::getDesiredIntakeAction, true)
    // );

    Command mainCommand =         RobotCommands.lockedHeadingDriving(
      drive,
      () -> -OI.m_driverController.getLeftY() * (OI.m_driverController.getRawButton(Button.kB.value) ? fineTuneSpeedMultiplier : 1),
      () -> -OI.m_driverController.getLeftX() * (OI.m_driverController.getRawButton(Button.kB.value) ? fineTuneSpeedMultiplier : 1),
      RobotStateManager::getDesiredFieldPose
    )
    .until(() -> { // proceed to scoring when desired tag is detected
      if(!RobotStateManager.getDesiredFieldPose().isOrientationOnly()) {
        for (int tagId : vision.getTagIds(0)) {
          if (tagId == RobotStateManager.getDesiredFieldPose().getTagId()) {
            return true;
          }
        }
      }
      return false;
    })
    .andThen(
      RobotCommands.scoreCoral(
        drive,
        vision,
        candle,
        elevator,
        pivot,
        intake,
        RobotStateManager::getReefHeight,
        RobotStateManager::getDesiredFieldPose
      )
    );

    new JoystickButton(OI.m_driverController, Button.kLeftBumper.value)
      .whileTrue(
        new ConditionalCommand(RobotCommands.dealgefy(drive, vision, candle, elevator, pivot, intake, RobotStateManager::getDesiredFieldPose), mainCommand, RobotStateManager::isAlignForAlgaePickup)
      ).onFalse(
        new RetractManipulator(elevator, pivot, RobotStateManager::isAlignForAlgaePickup)
      );

    // Switch to X pattern when X button is pressed
    new JoystickButton(OI.m_driverController, Button.kX.value).onTrue(Commands.runOnce(drive::stopWithX, drive));
    ////////////////////////////////////////////////////////////////////////////////////////////////////


    ////////////////////////////////// OPERATOR BOARD CONTROLLER //////////////////////////////////
    // OPBoard - Reef poses
    new JoystickButton(OI.m_operatorBoard, 1).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.A);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));
    new JoystickButton(OI.m_operatorBoard, 2).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.B);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));
    new JoystickButton(OI.m_operatorBoard, 3).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.C);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));
    new JoystickButton(OI.m_operatorBoard, 4).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.D);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));
    new JoystickButton(OI.m_operatorBoard, 5).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.E);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));
    new JoystickButton(OI.m_operatorBoard, 6).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.F);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));
    new JoystickButton(OI.m_operatorBoard, 7).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.G);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));
    new JoystickButton(OI.m_operatorBoard, 8).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.H);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));
    new JoystickButton(OI.m_operatorBoard, 9).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.I);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));
    new JoystickButton(OI.m_operatorBoard, 10).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.J);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));
    new JoystickButton(OI.m_operatorBoard, 11).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.K);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));
    new JoystickButton(OI.m_operatorBoard, 12).onTrue(Commands.runOnce(() -> {
      RobotStateManager.setCoralScoringPose(FieldPose.L);
      RobotStateManager.setRobotAction(RobotAction.REEF_ACTION);
    }));

    // OPBoard - Selecting desired reef height
    new JoystickButton(OI.m_operatorBoard, 19).onTrue(Commands.runOnce(() -> RobotStateManager.setReefHeight(ReefHeight.L4)));
    new JoystickButton(OI.m_operatorBoard, 20).onTrue(Commands.runOnce(() -> RobotStateManager.setReefHeight(ReefHeight.L3)));
    new JoystickButton(OI.m_operatorBoard, 21).onTrue(Commands.runOnce(() -> RobotStateManager.setReefHeight(ReefHeight.L2)));
    new JoystickButton(OI.m_operatorBoard, 22).onTrue(Commands.runOnce(() -> RobotStateManager.setReefHeight(ReefHeight.L1)));

    // OPBoard - Yellow buttons
    new JoystickButton(OI.m_operatorBoard, 13).onTrue(
      Commands.sequence(
        Commands.runOnce(() -> RobotStateManager.setRobotAction(RobotAction.INTAKE_STATION_LEFT)),
        new IntakeCoralCommand(intake, candle)
      )
    );
    new JoystickButton(OI.m_operatorBoard, 14).onTrue(
      Commands.sequence(
        Commands.runOnce(() -> RobotStateManager.setRobotAction(RobotAction.INTAKE_STATION_RIGHT)),
        new IntakeCoralCommand(intake, candle)
      )
    );
    new JoystickButton(OI.m_operatorBoard, 15).onTrue( // Stop intake
      Commands.runOnce(() -> new ManualIntakeCommand(intake, () -> 0).withTimeout(0.05).schedule())
    );
    new JoystickButton(OI.m_operatorBoard, 16).onTrue(Commands.runOnce(() -> RobotStateManager.setRobotAction(RobotAction.SCORE_BARGE_LEFT)));
    new JoystickButton(OI.m_operatorBoard, 17).onTrue(Commands.runOnce(() -> RobotStateManager.setRobotAction(RobotAction.SCORE_PROCESSOR)));
    new JoystickButton(OI.m_operatorBoard, 18).onTrue(Commands.runOnce(
        Constants.currentMode == Constants.Mode.SIM
        ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
        : () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    drive).ignoringDisable(true));

    // Switch whether score coral or intake alga
    new JoystickButton(OI.m_operatorBoard, 29).onTrue(
      Commands.sequence(
        Commands.runOnce(() -> RobotStateManager.setAlignForAlgaePickup(true)),
        new RetractManipulator(elevator, pivot, () -> true)
      )

    ).onFalse(
      Commands.sequence(
        Commands.runOnce(() -> RobotStateManager.setAlignForAlgaePickup(false)),
        new RetractManipulator(elevator, pivot, () -> false)
      )
    );
    ////////////////////////////////////////////////////////////////////////////////////////////////////


    ////////////////////////////////// MANUAL MANIPULATOR CONTROLLER //////////////////////////////////
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kY.value)
      .whileTrue(Commands.run(() -> elevator.setSpeed(0.22), elevator).onlyIf(pivot::isOutOfElevatorWay))
      .onFalse(Commands.runOnce(elevator::stop, elevator));
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kA.value)
      .whileTrue(Commands.run(() -> elevator.setSpeed(-0.18), elevator).onlyIf(pivot::isOutOfElevatorWay))
      .onFalse(Commands.runOnce(elevator::stop, elevator));
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kB.value)
      .whileTrue(Commands.run(() -> pivot.setSpeed(0.1), pivot))
      .onFalse(Commands.runOnce(pivot::stop, pivot));
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kX.value)
      .whileTrue(Commands.run(() -> pivot.setSpeed(-0.1), pivot))
      .onFalse(Commands.runOnce(pivot::stop, pivot));
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kRightBumper.value)
      .whileTrue(Commands.run(() -> intake.setSpeed(OI.m_mainpulatorControllerManual.getRawButton(Button.kStart.value) ? 1 : 0.5), intake))
      .onFalse(Commands.runOnce(intake::stop, intake));
    new JoystickButton(OI.m_mainpulatorControllerManual, Button.kLeftBumper.value)
      .whileTrue(Commands.run(() -> intake.setSpeed(OI.m_mainpulatorControllerManual.getRawButton(Button.kStart.value) ? -1 : -0.5), intake))
      .onFalse(Commands.runOnce(intake::stop, intake));
    ////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  /**
   * Setup SysId routines for the robot.
   */
  private void setupSysIdRoutines() {
    autoChooser.addOption("Feedforward Characterization Routine", DriveCharacterizationCommands.feedforwardCharacterization(drive));
    autoChooser.addOption("Effective Wheel Radius Routine", DriveCharacterizationCommands.wheelRadiusCharacterization(drive));

    try {
      SmartDashboard.putNumber("Current Sys Id Routine", sysIdRoutineId);
    } catch (Exception ignored) {}

    new JoystickButton(OI.m_sysIdRoutinesController, Button.kA.value).whileTrue(
      new ConditionalCommand(
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
        new ConditionalCommand(
          elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
          pivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
          () -> sysIdRoutineId == 2
        ),
        () -> sysIdRoutineId == 1
      )
    );

    new JoystickButton(OI.m_sysIdRoutinesController, Button.kB.value).whileTrue(
      new ConditionalCommand(
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
        new ConditionalCommand(
          elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
          pivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
          () -> sysIdRoutineId == 2
        ),
        () -> sysIdRoutineId == 1
      )
    );

    new JoystickButton(OI.m_sysIdRoutinesController, Button.kX.value).whileTrue(
      new ConditionalCommand(
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward),
        new ConditionalCommand(
          elevator.sysIdDynamic(SysIdRoutine.Direction.kForward),
          pivot.sysIdDynamic(SysIdRoutine.Direction.kForward),
          () -> sysIdRoutineId == 2
        ),
        () -> sysIdRoutineId == 1
      )
    );

    new JoystickButton(OI.m_sysIdRoutinesController, Button.kY.value).whileTrue(
      new ConditionalCommand(
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse),
        new ConditionalCommand(
          elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse),
          pivot.sysIdDynamic(SysIdRoutine.Direction.kReverse),
          () -> sysIdRoutineId == 2
        ),
        () -> sysIdRoutineId == 1
      )
    );

    new JoystickButton(OI.m_sysIdRoutinesController, Button.kRightBumper.value).whileTrue(
      Commands.runOnce(() -> {
        sysIdRoutineId++;
        try {
          SmartDashboard.putNumber("Current Sys Id Routine", sysIdRoutineId);
        } catch (Exception ignored) {}
      })
    );

    new JoystickButton(OI.m_sysIdRoutinesController, Button.kLeftBumper.value).whileTrue(
      Commands.runOnce(() -> {
        sysIdRoutineId--;
        try {
          SmartDashboard.putNumber("Current Sys Id Routine", sysIdRoutineId);
        } catch (Exception ignored) {}
      })
    );
  }

  /**
   * Register named commands for PathPlanner autos.
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("intake_coral", new IntakeCoralCommand(intake, candle));
    NamedCommands.registerCommand("l4_sequence", RobotCommands.scoreL4InPlace(drive, candle, elevator, pivot, intake));

    // Register named commands for reef height scoring poses
    NamedCommands.registerCommand("score_l4", Commands.sequence(
      Commands.runOnce(() -> RobotStateManager.setReefHeight(ReefHeight.L4)),
      new IntakeActionCommand(intake, candle, () -> IntakeAction.SCORE_L4, false)
    ));
    NamedCommands.registerCommand("score_l3", Commands.sequence(
      Commands.runOnce(() -> RobotStateManager.setReefHeight(ReefHeight.L4)),
      new IntakeActionCommand(intake, candle, () -> IntakeAction.SCORE_L3, false)
    ));
    NamedCommands.registerCommand("score_l2", Commands.sequence(
      Commands.runOnce(() -> RobotStateManager.setReefHeight(ReefHeight.L4)),
      new IntakeActionCommand(intake, candle, () -> IntakeAction.SCORE_L2, false)
    ));
    NamedCommands.registerCommand("score_l1", Commands.sequence(
      Commands.runOnce(() -> RobotStateManager.setReefHeight(ReefHeight.L4)),
      new IntakeActionCommand(intake, candle, () -> IntakeAction.SCORE_L1, false)
    ));

    // Elevator and pivot positioning
    NamedCommands.registerCommand("manipulator_retract", Commands.sequence(
      new RetractManipulator(elevator, pivot, () -> false)
    ));
    NamedCommands.registerCommand("manipulator_l4", Commands.sequence(
      Commands.runOnce(() -> RobotStateManager.setRobotAction(RobotAction.REEF_ACTION)),
      Commands.runOnce(() -> RobotStateManager.setReefHeight(ReefHeight.L4)),
      new PositionManipulator(elevator, pivot, () -> ElevatorPosition.L4, () -> PivotPosition.L4)
    ));
    NamedCommands .registerCommand("manipulator_l3", Commands.sequence(
      Commands.runOnce(() -> RobotStateManager.setRobotAction(RobotAction.REEF_ACTION)),
      Commands.runOnce(() -> RobotStateManager.setReefHeight(ReefHeight.L3)),
      new PositionManipulator(elevator, pivot, () -> ElevatorPosition.L3, () -> PivotPosition.L3)
    ));
    NamedCommands.registerCommand("manipulator_l2", Commands.sequence(
      Commands.runOnce(() -> RobotStateManager.setRobotAction(RobotAction.REEF_ACTION)),
      Commands.runOnce(() -> RobotStateManager.setReefHeight(ReefHeight.L2)),
      new PositionManipulator(elevator, pivot, () -> ElevatorPosition.L2, () -> PivotPosition.L2)
    ));

    for (FieldPose pose : FieldPose.values()) {
      if (pose.ordinal() >= 12) break; // make sure only reef coral scoring poses are registered

      String commandName = "align_" + pose.name();
      NamedCommands.registerCommand(commandName, Commands.sequence(
        Commands.runOnce(() -> RobotStateManager.setCoralScoringPose(pose)),
        RobotCommands.driveToPose(drive, vision, () -> pose)
      ));
    }
  }


  private void configureAutonEvents() {
    new EventTrigger("extend_manipulator_l4")
      .onTrue(
        RobotCommands.extendManipulators(elevator, pivot, () -> ElevatorPosition.L4, () -> PivotPosition.L4)
      );

    new EventTrigger("score_l4")
      .onTrue(
        new IntakeActionCommand(intake, candle, () -> IntakeAction.SCORE_L4, false)
      ).onFalse(
        RobotCommands.retractManipulator(elevator, pivot)
      );
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
