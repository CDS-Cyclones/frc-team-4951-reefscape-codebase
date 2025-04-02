package frc.robot;

import static frc.robot.Constants.DriveConstants.angleController;
import static frc.robot.Constants.DriveConstants.anglePIDCMaxSpeed;
import static frc.robot.Constants.DriveConstants.translationXController;
import static frc.robot.Constants.DriveConstants.translationXPIDCMaxSpeed;
import static frc.robot.Constants.DriveConstants.translationYController;
import static frc.robot.Constants.DriveConstants.translationYPIDCMaxSpeed;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.RobotStateConstants.CandleState;
import frc.robot.Constants.RobotStateConstants.ElevatorPosition;
import frc.robot.Constants.RobotStateConstants.FieldPose;
import frc.robot.Constants.RobotStateConstants.PivotPosition;
import frc.robot.Constants.RobotStateConstants.ReefHeight;
import frc.robot.commands.intake.IntakeActionCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Candle;
import frc.robot.subsystems.oi.OI;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.OIUtil;
import frc.robot.Constants.RobotStateConstants.IntakeAction;

public class RobotCommands {
  /**
   * Command to rumble a controller for a certain duration.
   *
   * @param controller
   * @param rumbleIntensity
   * @param rumbleDuration
   * @return Command
   */
  public static Command rumbleControllerForDuration(
    GenericHID controller,
    double rumbleIntensity,
    double rumbleDuration
  ) {
    return Commands.runEnd(
      () -> OI.rumbleController(controller, rumbleIntensity),
      () -> OI.rumbleController(controller, 0.0)
    ).withTimeout(rumbleIntensity);
  }

  /**
   * Command that turns leds on for a certain duration.
   *
   * @param candle
   * @param state
   * @param duration
   * @return Command
   */
  public static Command runLedsForDuration(
    Candle candle,
    CandleState state,
    double duration
  ) {
    return Commands.runEnd(
      () -> candle.setLEDs(state),
      () -> candle.setLEDs(CandleState.OFF)
    ).withTimeout(duration);
  }

  /**
   * Field-oriented driving with locked heading.
   *
   * @param drive
   * @param xSupplier
   * @param ySupplier
   * @param fieldPoseToLockHeadingToSupplier
   * @return Command
   */
  public static Command lockedHeadingDriving(
    Drive drive,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    Supplier<FieldPose> fieldPoseToLockHeadingToSupplier
  ) {
    return new FunctionalCommand(
      () -> {
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.reset();
      },
      () -> {
        boolean isFlipped =  DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
        double desiredHeadingRadians = fieldPoseToLockHeadingToSupplier.get().getDesiredPose().getRotation().toRotation2d().getRadians();
        double omega = angleController.calculate(drive.getRotation().getRadians(), desiredHeadingRadians);
        Translation2d linearVelocity = OIUtil.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        ChassisSpeeds speeds = new ChassisSpeeds(
          linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
          linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
          omega
        );

        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
      },
      interrupted -> {},
      () -> false,
      drive
    ).handleInterrupt(
      drive::stopWithX
    );
  }

  /**
   * Drive to a desired field pose.
   *
   * @param drive
   * @param vision
   * @param desiredFieldPoseSupplier
   * @return Command
   */
  public static Command driveToPose(
    Drive drive,
    Vision vision,
    Supplier<FieldPose> desiredFieldPoseSupplier
  ) {
    return new FunctionalCommand(
      () -> {
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.reset();
        translationXController.reset();
        translationYController.reset();
      },
      () -> {
        boolean isFlipped =  DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
        Pose3d pose = desiredFieldPoseSupplier.get().getDesiredPose();
        double omega = angleController.calculate(drive.getRotation().getRadians(), desiredFieldPoseSupplier.get().getDesiredRotation2d().getRadians());
        double velocityX = translationXController.calculate(drive.getPose().getTranslation().getX(), pose.getTranslation().getX());
        double velocityY = translationYController.calculate(drive.getPose().getTranslation().getY(), pose.getTranslation().getY());

        velocityX = MathUtil.applyDeadband(velocityX, 0.00);
        velocityY = MathUtil.applyDeadband(velocityY, 0.00);
        omega = MathUtil.applyDeadband(omega, 0.00);

        // clamp
        velocityX = MathUtil.clamp(velocityX, -translationXPIDCMaxSpeed.getAsDouble(), translationXPIDCMaxSpeed.getAsDouble());
        velocityY = MathUtil.clamp(velocityY, -translationYPIDCMaxSpeed.getAsDouble(), translationYPIDCMaxSpeed.getAsDouble());
        omega = MathUtil.clamp(omega, -anglePIDCMaxSpeed.getAsDouble(), anglePIDCMaxSpeed.getAsDouble());

        ChassisSpeeds speeds =  new ChassisSpeeds(
          isFlipped ? -velocityX : velocityX,
          isFlipped ? -velocityY : velocityY,
          omega
        );

        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
      },
      interrupted -> {},
      () -> angleController.atSetpoint() && translationXController.atSetpoint() && translationYController.atSetpoint(),
      drive, vision
    );
  }

  /**
   * Conditional commands that assures the pivot is out of the way of the elevator's way.
   * Should nearly always be used before raising the elevator.
   *
   * @param elevator
   * @param pivot
   * @return Command
   */
  public static Command assureElevatorIsPivotClear(
    Elevator elevator,
    Pivot pivot
  ) {
    return new ConditionalCommand(
      Commands.none(),
      Commands.sequence(
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
        Commands.waitUntil(() -> pivot.isAtPosition(PivotPosition.ELEVATOR_CLEAR))
      ),
      pivot::isOutOfElevatorWay
    );
  }

  /**
   * Command to extend the manipulators
   *
   * @param elevator
   * @param pivot
   * @return Command
   */
  public static Command extendManipulators(
    Elevator elevator,
    Pivot pivot,
    Supplier<ElevatorPosition> elevatorPositionSupplier,
    Supplier<PivotPosition> pivotPositionSupplier
  ) {
    return
    Commands.sequence(
      assureElevatorIsPivotClear(elevator, pivot),
      Commands.parallel(
        elevator.moveToPosition(pivot, elevatorPositionSupplier),
        pivot.moveToPosition(pivotPositionSupplier)
      )
    );
  }

  /**
   * Command to retract the manipulators.
   *
   * @param elevator
   * @param pivot
   * @return Command
   */
  public static Command retractManipulator(
    Elevator elevator,
    Pivot pivot
  ) {
    return
    Commands.sequence(
      assureElevatorIsPivotClear(elevator, pivot),
      Commands.parallel(
        elevator.moveToPosition(pivot, () -> ElevatorPosition.DOWN),
        pivot.moveToPosition(() -> PivotPosition.INTAKE_READY)
      )
    );
  }

  public static Command scoreL4InPlace(
    Drive drive,
    Candle candle,
    Elevator elevator,
    Pivot pivot,
    Intake intake
  ) {
    return Commands.sequence(
      assureElevatorIsPivotClear(elevator, pivot),
      elevator.moveToPosition(pivot, () -> ElevatorPosition.L4),
      pivot.moveToPosition(() -> PivotPosition.L4),
      new IntakeActionCommand(intake, candle, () -> IntakeAction.SCORE_L4, false),
      retractManipulator(elevator, pivot)
    );
  }

  /**
   * Drive up and score the coral to the desired reef height.
   * Works for L4, L3, and L2.
   * Retracts and stops on interrupt.
   *
   * @param drive
   * @param vision
   * @param candle
   * @param elevator
   * @param pivot
   * @param intake
   * @param reefHeightSupplier
   * @param reefPoseSupplier
   * @return Command
   */
  public static Command scoreCoral(
    Drive drive,
    Vision vision,
    Candle candle,
    Elevator elevator,
    Pivot pivot,
    Intake intake,
    Supplier<ReefHeight> reefHeightSupplier,
    Supplier<FieldPose> reefPoseSupplier
  ) {
    return Commands.sequence(
      Commands.runOnce(() -> candle.setLEDs(CandleState.SCORING), candle),
      Commands.parallel(
        rumbleControllerForDuration(OI.m_driverController, 0.5, 1),
        driveToPose(drive, vision, reefPoseSupplier),
        Commands.sequence(
          assureElevatorIsPivotClear(elevator, pivot),
          Commands.parallel(
            elevator.moveToPosition(pivot, () -> RobotStateManager.getElevatorPositionForSpecificReefHeight(reefHeightSupplier)),
            pivot.moveToPosition(() -> RobotStateManager.getPivotPositionForSpecificReefHeight(reefHeightSupplier))
          )
        )
      ),
      new IntakeActionCommand(intake, candle, () -> RobotStateManager.getIntakeActionForSpecificReefHeight(reefHeightSupplier), false),
      Commands.parallel(
        runLedsForDuration(candle, CandleState.SCORED, 1),
        rumbleControllerForDuration(OI.m_driverController, 0.5, 1)
      )
    ).finallyDo(() -> {
      Commands.parallel(
        Commands.runOnce(drive::stopWithX, drive),
        retractManipulator(elevator, pivot),
        Commands.runOnce(intake::stop),
        Commands.runOnce(() -> candle.setLEDs(CandleState.OFF), candle)
      );
    });
  }

    /**
     * Drive up to reef, knock down the algae on the way up and then score L4.
     * Works for L4 only.
     * Does retract manipulator.
     *
     * @param drive
     * @param vision
     * @param candle
     * @param elevator
     * @param pivot
     * @param intake
     * @param xSupplier
     * @param ySupplier
     * @param reefPoseSupplier pose for scoring the coral
     * @return Command
     */
    public static Command dealgaefyThenScoreL4(
      Drive drive,
      Vision vision,
      Candle candle,
      Elevator elevator,
      Pivot pivot,
      Intake intake,
      Supplier<FieldPose> reefPoseSupplier
    ) {
      return Commands.sequence(
        Commands.runOnce(() -> candle.setLEDs(CandleState.SCORING), candle),
          Commands.parallel(
            rumbleControllerForDuration(OI.m_driverController, 0.5, 1),
            driveToPose(drive, vision, () -> RobotStateManager.RobotState.getCorrespondingPose(reefPoseSupplier.get()))
          ),
          assureElevatorIsPivotClear(elevator, pivot),
          pivot.moveToPosition(() -> PivotPosition.DEALGAEFY),
          elevator.moveToPosition(pivot, () -> ElevatorPosition.L4),
          pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
          driveToPose(drive, vision, reefPoseSupplier),
          pivot.moveToPosition(() -> PivotPosition.L4),
          new IntakeActionCommand(intake, candle, () -> RobotStateManager.getIntakeActionForSpecificReefHeight(() -> ReefHeight.L4), true),
          Commands.parallel(
            runLedsForDuration(candle, CandleState.SCORED, 1),
            rumbleControllerForDuration(OI.m_driverController, 0.5, 1)
          )
        ).handleInterrupt(() -> {
          Commands.parallel(
            Commands.runOnce(drive::stopWithX, drive),
            retractManipulator(elevator, pivot),
            Commands.runOnce(intake::stop),
            Commands.runOnce(() -> candle.setLEDs(CandleState.OFF), candle)
          );
        });
    }
}
