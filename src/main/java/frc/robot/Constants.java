// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.TunableValues.TunableNum;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class MotorConstants {
    public static final double neoKv = 473;  // https://docs.revrobotics.com/brushless/neo/v1.1#motor-specifications
  }

  public static final class DriveConstants {
    public static final double maxSpeedMetersPerSecAuton = 2.5;
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(27.0);
    public static final double wheelBase = Units.inchesToMeters(32.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(Math.PI);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);

    // Device CAN IDs
    public static final int pigeon2CanId = 38;
    public static final String pigeon2CanBus = "pigeonbus";

    public static final int frontLeftDriveCanId = 21;
    public static final int backLeftDriveCanId = 27;
    public static final int frontRightDriveCanId = 23;
    public static final int backRightDriveCanId = 25;

    public static final int frontLeftTurnCanId = 22;
    public static final int backLeftTurnCanId = 28;
    public static final int frontRightTurnCanId = 24;
    public static final int backRightTurnCanId = 26;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 80;
    public static final double wheelRadiusMeters = 0.040704575067725526;
    public static final double driveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final TunableNum driveKp = new TunableNum("Drive/drive/p", 0.0);
    public static final TunableNum driveKd = new TunableNum("Drive/drive/d", 0.0);
    public static final double driveKs = 0.18406757595208734;
    public static final double driveKv = 0.09511381538864068;
    public static final double driveSimP = 0.0;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.04038;
    public static final double driveSimKv = 0.11972;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 40;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final TunableNum turnKp = new TunableNum("Drive/turn/p", 1.0);
    public static final TunableNum turnKd = new TunableNum("Drive/turn/d", 0.05);
    public static final double turnSimP = 10;
    public static final double turnSimD = 0.2;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PID controllers
    public static final TunableNum anglePIDCKp = new TunableNum("Drive/controllers/angle/P", 3.2);
    public static final TunableNum anglePIDCKi = new TunableNum("Drive/controllers/angle/I", 0);
    public static final TunableNum anglePIDCKd = new TunableNum("Drive/controllers/angle/D", 0);
    public static final TunableNum anglePIDCTolerance = new TunableNum("Drive/controllers/angle/tolerance", 0.02);
    public static final TunableNum anglePIDCMaxSpeed = new TunableNum("Drive/controllers/angle/maxSpeed", 6);  // in radians per second

    public static final TunableNum translationXPIDCKp = new TunableNum("Drive/controllers/translation/x/P", 3);
    public static final TunableNum translationXPIDCKi = new TunableNum("Drive/controllers/translation/x/I", 0);
    public static final TunableNum translationXPIDCKd = new TunableNum("Drive/controllers/translation/x/D", 0);
    public static final TunableNum translationXPIDCTolerance = new TunableNum("Drive/controllers/translation/x/tolerance", 0.05);
    public static final TunableNum translationXPIDCMaxSpeed = new TunableNum("Drive/controllers/translation/x/maxSpeed", 3);  // in meters per second

    public static final TunableNum translationYPIDCKp = new TunableNum("Drive/controllers/translation/y/P", 3);
    public static final TunableNum translationYPIDCKi = new TunableNum("Drive/controllers/translation/y/I", 0);
    public static final TunableNum translationYPIDCKd = new TunableNum("Drive/controllers/translation/y/D", 0);
    public static final TunableNum translationYPIDCTolerance = new TunableNum("Drive/controllers/translation/y/tolerance", 0.05);
    public static final TunableNum translationYPIDCMaxSpeed = new TunableNum("Drive/controllers/translation/y/maxSpeed", 3);  // in meters per second

    public static final PIDController angleController = new PIDController(
      anglePIDCKp.getAsDouble(),
      anglePIDCKi.getAsDouble(),
      anglePIDCKd.getAsDouble()
    );
    public static final PIDController translationXController = new PIDController(
      translationXPIDCKp.getAsDouble(),
      translationXPIDCKi.getAsDouble(),
      translationXPIDCKd.getAsDouble()
    );
    public static final PIDController translationYController = new PIDController(
      translationYPIDCKp.getAsDouble(),
      translationYPIDCKi.getAsDouble(),
      translationYPIDCKd.getAsDouble()
    );

    // Drive command configuration
    public static final double fineTuneSpeedMultiplier = 0.4;

    // PathPlanner configuration
    public static final double robotMassKg = 59.0531907;
    public static final double robotMOI = 7.31266;
    public static final double wheelCOF = 1.6;  // https://www.chiefdelphi.com/t/wildstang-robotics-program-team-111-and-112-build-blog-2025/477716/36
    public static final PIDConstants ppDrivePID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ppTurnPID = new PIDConstants(5.0, 0.0, 0.0);
    public static final RobotConfig ppConfig =
      new RobotConfig(
        robotMassKg,
        robotMOI,
        new ModuleConfig(
          wheelRadiusMeters,
          maxSpeedMetersPerSecAuton,
          wheelCOF,
          driveGearbox.withReduction(driveMotorReduction),
          driveMotorCurrentLimit,
          1
        ),
        moduleTranslations
      );

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
      .withCustomModuleTranslations(moduleTranslations)
      .withRobotMass(Kilograms.of(robotMassKg))
      .withGyro(COTS.ofPigeon2())
      .withSwerveModule(new SwerveModuleSimulationConfig(
        driveGearbox,
        turnGearbox,
        driveMotorReduction,
        turnMotorReduction,
        Volts.of(0.1),
        Volts.of(0.1),
        Meters.of(wheelRadiusMeters),
        KilogramSquareMeters.of(0.02),
        wheelCOF
      )
    );
  }

  public static final class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static String limelightFrontName = "limelight";
    public static String limelightBackName = "limelight-back";
    public static String cameraNameSim = "simCam";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d botToCamTransformSim = new Transform3d(
      new Translation3d(-.3, 0, 0), // X is forward in m, z is up in m
      new Rotation3d(0, 0, 0)  // facing forward
    );

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {1.0};

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class OIConstants {
    // Joystick ports
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorBoardPort = 1;
    public static final int kOperatorControllerPort = 2;
    public static final int kOperatorControllerManualPort = 3;
    public static final int kSysIDRoutinesControllerPort = 4;

    // Joystick axis deadband
    public static final double kJoystickAxisDeadband = 0.1;
  }

  public static final class ManipulatorConstants {
    // CAN IDs
    public static final int elevatorMotor1Id = 31;
    public static final int elevatorMotor2Id = 32;
    public static final int pivotMotorId = 56;
    public static final int intakeMotorId = 57;
    public static final int coralInflowCanrangeCanId = 41;
    public static final int coralOutflowCanrangeCanId = 42;
    public static final String coralInflowCanrangeCanBus = "pigeonbus";
    public static final String coralOutflowCanrangeCanBus = "pigeonbus";

    // Elevator constants
    public static final boolean elevatorMotorInverted = false;
    public static final boolean elevatorMotorFollowerInverted = false;
    public static final double elevatorMinPosition = 0.0;
    public static final double elevatorMaxPosition = 1.8;
    public static final TunableNum elevatorKp = new TunableNum("Elevator/P", 2.85);
    public static final TunableNum elevatorKd = new TunableNum("Elevator/D", 0.0);
    public static final double elevatorKff = 1 / MotorConstants.neoKv;  // not arbFF, inverse of motor specific Kv value
    public static final double elevatorKs = 0.58291;
    public static final double elevatorKg = 0.62411;
    public static final double elevatorKv = 3.2946;
    public static final double elevatorKa = 0.62268;
    public static final double elevatorDistancePerRevolution = Units.inchesToMeters(63)/52.25;
    public static final double elevatorVelocityMetersPerSecond = elevatorDistancePerRevolution / 60.0;
    public static final TunableNum elevatorMinSpeed = new TunableNum("Elevator/MinSpeed", -0.5); // max speed going down
    public static final TunableNum elevatorMaxSpeed = new TunableNum("Elevator/MaxSpeed", 0.61); // max speed going up
    public static final TunableNum elevatorPositionTolerance = new TunableNum("Elevator/PositionTolerance", 0.05);

    // Pivot constants
    public static final boolean pivotMotorInverted = true;
    public static final boolean pivotAbsoluteEncoderInverted = false;
    public static final double pivotOffsetFromHorizontal = -2.899 + Math.PI / 2;
    public static final double pivotMinPosition = -1;
    public static final double pivotMaxPosition = 2.28;
    public static final double pivotMinPositionForElevatorMovement = 2;
    public static final TunableNum pivotKp = new TunableNum("Pivot/P", 0.36);
    public static final TunableNum pivotKd = new TunableNum("Pivot/D", 0.07);
    public static final double pivotKff =  1 / MotorConstants.neoKv;  // not arbFF, inverse of motor specific Kv value
    public static final double pivotKs = 0.29184;
    public static final double pivotKg = 0.37151;
    public static final double pivotKv = 1.6051;
    public static final double pivotKa = 0;
    public static final double pivotAbsoluteEncoderRadiansPerRevolution = Units.degreesToRadians(90) / 0.25;
    public static final double pivotAbsoluteEncoderAngularVelocityRadiansPerSecond = pivotAbsoluteEncoderRadiansPerRevolution / 60.0;
    public static final double pivotRelativeEncoderRadiansPerRevolution = Units.degreesToRadians(90) / 8.0;
    public static final double pivotRelativeEncoderAngularVelocityRadiansPerSecond = pivotRelativeEncoderRadiansPerRevolution / 60.0;
    public static final TunableNum pivotMinSpeed = new TunableNum("Pivot/MinSpeed", -0.17);  // max speed going out
    public static final TunableNum pivotMaxSpeed = new TunableNum("Pivot/MaxSpeed", 0.18);  // max speed going in
    public static final TunableNum pivotPositionTolerance = new TunableNum("Pivot/PositionTolerance", 0.01);

    // Intake constants
    public static final boolean intakeMotorInverted = false;
    public static final double coralCanrangeDistanceThreshold = 0.1;

    // Elevator simulation constants
    public static final double elevatorGearing = 1.0;        // NOT TUNED
    public static final double elevatorCarriageMass = 1.0;   // NOT TUNED
    public static final double elevatorDrumRadius = 0.0254;  // NOT TUNED
    public static final double elevatorMinHeightMetres = 0.0;
    public static final double elevatorMaxHeightMetres = 3;
  }

  public static final class CandleConstants {
    public static final int candleId = 40;
    public static final String candleBus = "pigeonbus";
    public static double candleBrightness = 1;

    public static int ledPWMPort = 0;
    public static int ledCount = 60;
  }

  public static final class RobotStateConstants {
    // Constants for field poses
    private static final double inFrontOfTag = Units.inchesToMeters(23);
    private static final double rightOfTag = Units.inchesToMeters(6.6);
    private static final double leftOfTag = -Units.inchesToMeters(7.15);

    private static final double inFrontOfTagSim = 0.4;

    /** A tunable pivot position */
    public static final TunableNum tunablePivotPosition = new TunableNum("Pivot/TuneablePosition", 0.0);

    /** A tunable elevator position */
    public static final TunableNum tunableElevatorPosition = new TunableNum("Elevator/TuneablePosition", 0.0);

    /** A tunable intake speed */
    public static final TunableNum tunableIntakeSpeed = new TunableNum("Intake/TunableSpeed", 0.0);

    /** A tunable intake time */
    public static final TunableNum tunableIntakeTime = new TunableNum("Intake/TunableTime", 0.0);

    /**
     * An enum to represent all desired robot actions.
     */
    public static enum RobotAction {
      REEF_ACTION,  // Either score coral or intake an alga from the reef
      SCORE_BARGE_LEFT,  // Score an alga in the barge on the left
      SCORE_BARGE_RIGHT,  // Score an alga in the barge on the right
      SCORE_PROCESSOR,  // Score an alga in the processor
      INTAKE_STATION_LEFT,  // Intake a coral from the station on the left
      INTAKE_STATION_RIGHT,  // Intake a coral from the station on the right
    }

    /**
     * An enum to represent all desired reef heights.
     */
    public static enum ReefHeight {
      L1,
      L2,
      L3,
      L4
    }

    /**
     * An enum to represent all desired pivot positions.
     */
    @RequiredArgsConstructor
    public static enum PivotPosition {
      INTAKE_READY(2.27),
      ELEVATOR_CLEAR(1.7), // when empty
      ELEVATOR_CLEAR_WITH_ALGA(0.6),
      DEALGAEFY(1.5),
      L1(0.0),
      L2(1.79),
      L3(1.79),
      L4(1.86),
      REEF_ALGA_L2(-.5),
      REEF_ALGA_L3(-.5),
      BARGE_START(0.95),
      BARGE_END(1.43),
      PROCESSOR(-0.87),
      FULLY_OUT(-1),
      TUNABLE(Double.NaN);  // Special value for tunable position

      private final double position;

      public double getAsDouble() {
        if (this == TUNABLE) {
          return tunablePivotPosition.getAsDouble();
        }
        return position;
      }

      @Override
      public String toString() {
        return name() + " (" + getAsDouble() + ")";
      }
    }

    /**
     * An enum to represent all desired elevator positions.
     */
    @RequiredArgsConstructor
    public static enum ElevatorPosition {
      DOWN(0.0),
      L1(0.0),
      L2(0.31),
      L3(0.82),
      L4(1.58),
      REEF_ALGA_L2(0.6),
      REEF_ALGA_L3(0.95),
      BARGE(1.79),
      PROCESSOR(0.0),
      TUNABLE(Double.NaN);  // Special value for tunable position

      private final double position;

      public double getAsDouble() {
        if (this == TUNABLE) {
          return tunableElevatorPosition.getAsDouble();
        }
        return position;
      }

      @Override
      public String toString() {
        return name() + " (" + getAsDouble() + ")";
      }
    }

    /**
     * An enum to represent all desired field poses of the robot.
     */
    @RequiredArgsConstructor
    public static enum FieldPose {
      // CORAL SCORING POSES MUST REMAIN FIRST 12!
      A(21, 10, inFrontOfTag, rightOfTag, Math.PI, false),
      B(21, 10, inFrontOfTag, leftOfTag, Math.PI, false),
      C(22, 9, inFrontOfTag, rightOfTag, Math.PI, false),
      D(22, 9, inFrontOfTag, leftOfTag, Math.PI, false),
      E(17, 8, inFrontOfTag, rightOfTag, Math.PI, false),
      F(17, 8, inFrontOfTag, leftOfTag, Math.PI, false),
      G(18, 7, inFrontOfTag, rightOfTag, Math.PI, false),
      H(18, 7, inFrontOfTag, leftOfTag, Math.PI, false),
      I(19, 6, inFrontOfTag, rightOfTag, Math.PI, false),
      J(19, 6, inFrontOfTag, leftOfTag, Math.PI, false),
      K(20, 11, inFrontOfTag, rightOfTag, Math.PI, false),
      L(20, 11, inFrontOfTag, leftOfTag, Math.PI, false),
      Z1(21, 10, inFrontOfTag, 0, Math.PI, false),
      Z2(22, 9, inFrontOfTag, 0, Math.PI, false),
      Z3(17, 8, inFrontOfTag, 0, Math.PI, false),
      Z4(18, 7, inFrontOfTag, 0, Math.PI, false),
      Z5(19, 6, inFrontOfTag, 0, Math.PI, false),
      Z6(20, 11, inFrontOfTag, 0, Math.PI, false),
      STATION_LEFT(13, 1, 0, 0, 0, true),
      STATION_RIGHT(12, 2, 0, 0, 0, true),
      BARGE_LEFT(14, 5, 0, 0, Math.PI, true),
      BARGE_RIGHT(15, 4, 0, 0, Math.PI, true),
      PROCESSOR(16, 3, 0, 0, Math.PI, true);

      private final int tagBlueId;
      private final int tagRedId;
      private final double away;
      private final double side;
      private final double rotation; // in radians
      private final boolean orientationOnly;

      /**
       * Return ID of the tag the pose is relative to.
       * @return The ID of the tag.
       */
      public int getTagId() {
        return DriverStation.getAlliance().get() == Alliance.Red ? tagRedId : tagBlueId;
      }

      /**
       * Return the pose of the tag the pose is relative to.
       * @return {@link Pose3d} of the tag.
       */
      public Pose3d getTagPose() {
        return Constants.VisionConstants.aprilTagLayout.getTagPose(getTagId()).get();
      }

      /**
       * Return the desired pose of the robot.
       * If orientationOnly is true, this will return null.
       * @return {@link Pose3d} of the robot.
       */
      public Pose3d getDesiredPose() {
        return getDesiredPose(false, false, false);
      }

      /**
       * Return the desired pose of the robot.
       * If orientationOnly is true, this will return null.
       * @param ignoreForwards Whether to ignore the forwards offset.
       * @param ignoreSideways Whether to ignore the sideways offset.
       * @param ignoreRotation Whether to ignore the rotation.
       * @return {@link Pose3d} of the robot.
       */
      public Pose3d getDesiredPose(boolean ignoreForwards, boolean ignoreSideways, boolean ignoreRotation) {
        Pose3d tagPose = getTagPose();
        double tagAngle = tagPose.getRotation().toRotation2d().getRadians();
        double tagX = tagPose.getTranslation().getX();
        double tagY = tagPose.getTranslation().getY();

        // Ensure the angle is between 0 and 2pi
        if (tagAngle < 0) {
          tagAngle = 2 * Math.PI + tagAngle;
        }

        double cos = Math.cos(tagAngle);
        double sin = Math.sin(tagAngle);

        double newX = tagX;
        double newY = tagY;
        Pose3d newPose;

        switch (Constants.currentMode) {
          case SIM:
            newX += inFrontOfTagSim * cos;
            newY += inFrontOfTagSim * sin;
          default:
            newX += away * cos;
            newY += away * sin;
        }

        // now do transformation to the left or right of the tag
        newX += side * -sin;
        newY += side * cos;

        newPose = new Pose3d(new Translation3d(
          ignoreForwards ? tagX : newX,
          ignoreForwards ? tagY : newY,
          0
        ), new Rotation3d(
          0,
          0,
          ignoreRotation ? tagAngle : tagAngle + rotation
        ));

        return newPose;
      }


      /**
       * Return the desired rotation of the robot.
       * @return {@link Rotation2d} of the robot.
       */
      public Rotation2d getDesiredRotation2d() {
        return getDesiredPose().getRotation().toRotation2d();
      }

      /**
       * Return whether the desired pose is orientation only meaning the robot should only
       * rotate to the desired angle and not move.
       *
       * @return True if the desired pose is orientation only.
       */
      public boolean isOrientationOnly() {
        return orientationOnly;
      }

      @Override
      public String toString() {
        return name();
      }
    }


    /**
     * An enum to represent the states of the CANdle.
     */
    @RequiredArgsConstructor
    public enum CandleState {
      OFF(255, 215, 0, 0.57),  // gold
      TARGET_FOUND(255, 188, 0, 0.65), // orange
      SCORING(127, 0, 255, 0.91), // violet
      SCORED(128, 255, 0, -0.47),  // green
      WAITIING_FOR_CORAL(207, 16, 32, -0.39),  // lava
      CORAL_DETECTED(143, 0, 255, 0.91);  // violet

      @Getter private final int red;
      @Getter private final int green;
      @Getter private final int blue;
      @Getter private final double blinkingPwmValue;

      @Override
      public String toString() {
        return "RGB: (" + red + ", " + green + ", " + blue + ")";
      }
    }

    /**
     * An enum to represent all desired intake actions.
     */
    @AllArgsConstructor
    public static enum IntakeAction {
      NONE(0.0, 0.0),
      OCCUPIED(0.0, 0.0),  // Special value for when the intake is occupied by another command
      SCORE_L1(0.2, 3.0),
      SCORE_L2(1.0, 1.1),
      SCORE_L3(1.0, 1.1),
      SCORE_L4(1.0, 1.2),
      SCORE_BARGE(1.0, 2.0),
      SCORE_PROCESSOR(1.0, 2.0),
      INTAKE_REEF_ALGA(-0.9, 1.5),
      OUTTAKE_REEF_ALGAE(1, 1.0),
      INTAKE_CORAL(0.56, 1.2),  // Time is redundant; uses Canrange sensor
      TUNABLE(Double.NaN, Double.NaN); // Special values for tunable speed and duration

      private final double speed;
      private final Double time;

      /**
       * Returns the speed at which this action should run.
       *
       * @return The speed at which this action should run.
       */
      public double getSpeed() {
        if (this == TUNABLE) {
          return tunableIntakeSpeed.getAsDouble();
        }
        return speed;
      }

      /**
       * Returns the time for which this action should run.
       *
       * @return The time for which this action should run.
       */
      public Double getTime() {
        if (this == TUNABLE) {
          return tunableIntakeTime.getAsDouble();
        }
        return time;
      }

      @Override
      public String toString() {
        return name() + " (" + getSpeed() + ", " + getTime() + ")";
      }
    }
  }
}
