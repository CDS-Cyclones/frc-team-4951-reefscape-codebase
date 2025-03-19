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
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

  public static final class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(27.0);  // TODO measure
    public static final double wheelBase = Units.inchesToMeters(32.5);  // TODO measure
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2.0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);

    // Device CAN IDs
    public static final int pigeon2CanId = 9;
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
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);  // TODO Run Drive Wheel Radius Characterization
    public static final double driveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final TunableNum driveKp = new TunableNum("Drive/drive/p", 0.0);  // TODO tune
    public static final TunableNum driveKd = new TunableNum("Drive/drive/d", 0.0);  // TODO tune
    public static final double driveKs = 0.0;  // TODO Run Drive Wheel Radius Characterization
    public static final double driveKv = 0.0;  // TODO Run Drive Wheel Radius Characterization
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
    public static final TunableNum turnKp = new TunableNum("Drive/turn/p", 2.0);  // TODO tune
    public static final TunableNum turnKd = new TunableNum("Drive/turn/d", 0.1);  // TODO tune
    public static final double turnSimP = 10;
    public static final double turnSimD = 0.2;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // Configuration for PID controllers
    public static final TunableNum anglePIDCKp = new TunableNum("Drive/PIDController/angle/p", 6.0);  // TODO tune
    public static final TunableNum anglePIDCKd = new TunableNum("Drive/PIDController/angle/d", 0.4);  // TODO tune
    public static final TunableNum anglePIDCMaxVel = new TunableNum("Drive/PIDController/angle/maxVel", 8);  // TODO tune // Radians per second
    public static final TunableNum anglePIDCMaxAccel = new TunableNum("Drive/PIDController/angle/maxAccel", 20);  // TODO tune // Radians per second squared
    public static final TunableNum anglePIDTolerance = new TunableNum("Drive/PIDController/angle/errorTolerance", 0.05);  // TODO tune
    public static final TunableNum translationPIDCKp = new TunableNum("Drive/PIDController/translation/p", 4);  // TODO tune
    public static final TunableNum translationPIDCKd = new TunableNum("Drive/PIDController/translation/d", 0.1);  // TODO tune
    public static final TunableNum translationPIDCMaxVel = new TunableNum("Drive/PIDController/translation/maxVel", 6);  // TODO tune // Meters per second
    public static final TunableNum translationPIDCMaxAccel = new TunableNum("Drive/PIDController/translation/maxAccel", 15);  // TODO tune // Meters per second squared
    public static final TunableNum translationPIDTolerance = new TunableNum("Drive/PIDController/translation/errorTolerance", 0.05);  // TODO tune

    // Drive command configuration
    public static final double fineTuneSpeedMultiplier = 0.35;

    // PathPlanner configuration
    public static final double robotMassKg = 52;  // TODO measure
    public static final double robotMOI = 6.883;  // TODO calculate w/ formula after running SysId
    public static final double wheelCOF = 1.2;
    public static final PIDConstants ppDrivePID = new PIDConstants(5.0, 0.0, 0.0);  // TODO tune
    public static final PIDConstants ppTurnPID = new PIDConstants(5.0, 0.0, 0.0);  // TODO tune
    public static final RobotConfig ppConfig =
      new RobotConfig(
        robotMassKg,
        robotMOI,
        new ModuleConfig(
          wheelRadiusMeters,
          maxSpeedMetersPerSec,
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

    public static String cameraName = "limelight";
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
    public static final int coralCanrangeCanId = 40;
    public static final int algaCanrangeCanId = 41;

    // Motor configurations

    public static final SparkBaseConfig pivotMotorConfig = new SparkMaxConfig()
      .smartCurrentLimit(80)
      .secondaryCurrentLimit(90)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(true);
    public static final SparkBaseConfig intakeWheelsMotorConfig = new SparkMaxConfig()
      .smartCurrentLimit(80)
      .secondaryCurrentLimit(90)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(false);

    public static final double neoKv = 473;  // https://docs.revrobotics.com/brushless/neo/v1.1#motor-specifications

    // Elevator constants
    public static final boolean elevatorMotorInverted = false;
    public static final boolean elevatorMotorFollowerInverted = false;
    public static final double elevatorMinPosition = 0.0;                                                                                // TODO figure out
    public static final double elevatorMaxPosition = 1.0;                                                                                // TODO figure out
    public static final TunableNum elevatorKp = new TunableNum("Elevator/P", 0.0);                                     // TODO tune
    public static final TunableNum elevatorKd = new TunableNum("Elevator/D", 0.0);                                     // TODO tune
    public static final double elevatorKff =  1 / neoKv;  // not arbFF, inverse of motor specific Kv value
    public static final double elevatorKs = 0.0;                                                                                         // TODO figure out
    public static final double elevatorKg = 0.0;                                                                                         // TODO figure out - SysId
    public static final double elevatorKv = 0.0;                                                                                         // TODO figure out - SysId
    public static final double elevatorKa = 0.0;                                                                                         // TODO figure out - SysId
    public static final double elevatorDistancePerRevolution = Units.inchesToMeters(1.0/1.0);                                            // TODO figure out - SysId
    public static final double elevatorVelocityMetersPerSecond = elevatorDistancePerRevolution / 60.0;                                   // TODO figure out
    public static final TunableNum elevatorMinSpeed = new TunableNum("Elevator/MinSpeed", -0.3); // max speed going down            // TODO tune
    public static final TunableNum elevatorMaxSpeed = new TunableNum("Elevator/MinSpeed", 0.3); // max speed going up  // TODO tune
    public static final TunableNum elevatorPositionTolerance = new TunableNum("Elevator/PositionTolerance", 0.05);     // TODO tune

    // Pivot constants
    public static final double pivotMinPosition = 0.0;  // TODO figure out
    public static final double pivotMaxPosition = 1.0;  // TODO figure out
    public static final TunableNum pivotMaxSpeed = new TunableNum("Pivot/MaxSpeed", 0.5);  // TODO tune
    public static final TunableNum pivotMaxAcceleration = new TunableNum("Pivot/MaxAcceleration", 0.5);  // TODO tune
    public static final TunableNum pivotKp = new TunableNum("Pivot/P", 0.5);  // TODO tune
    public static final TunableNum pivotKd = new TunableNum("Pivot/D", 0.0);  // TODO tune
    public static final double pivotKs = 0.0;  // TODO figure out
    public static final double pivotKg = 0.0;  // TODO figure out
    public static final double pivotKv = 0.0;  // TODO figure out
    public static final double pivotKa = 0.0;  // TODO figure out
    public static final TunableNum pivotPIDTolerance = new TunableNum("Pivot/ErrorTolerance", 0.0);  // TODO tune
    public static final double pivotMinPositionForElevatorMovement = 10000000;  // TODO figure out

    // Intake constants
    public static final double coralCanrangeDistanceThreshold = 0.05; // in m TODO figure this out
    public static final double algaCanrangeDistanceThreshold = 0.2; // in m TODO figure this out

    // Elevator simulation constants
    public static final double elevatorGearing = 1.0;
    public static final double elevatorCarriageMass = 1.0;
    public static final double elevatorDrumRadius = 0.0254;
    public static final double elevatorMinHeightMetres = 0.0;
    public static final double elevatorMaxHeightMetres = 2.2;

  }

  public static final class CandleConstants {
    public static final int candleId = 40;
    public static final String candleBus = "pigeonbus";
    public static double candleBrightness = 0.5;
  }

  public static final class RobotStateConstants {
    // Constants for field poses
    private static final double inFrontOfTag = 0.05;
    private static final double inFrontOfTagSim = 0.4;
    private static final double leftOfTag = -0.2;
    private static final double rightOfTag = 0.2;

    /** A tunable pivot position */
    public static final TunableNum tunablePivotPosition = new TunableNum("Pivot/TuneablePosition", 0.0);
    
    /** A tunable elevator position */
    public static final TunableNum tunableElevatorPosition = new TunableNum("Elevator/TuneablePositionBababui", 0.0);
    
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
      INTAKE_READY(0.0),
      ELEVATOR_CLEAR(0.0),
      L1(0.0),
      L2(0.0),
      L3(0.0),
      L4(0.0),
      REEF_ALGA(0.0),
      BARGE(0.0),
      PROCESSOR(0.0),
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
      L2(0.0),
      L3(0.0),
      L4(0.0),
      REEF_ALGA(0.0),
      BARGE(0.0),
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

        newPose = new Pose3d(new Translation3d(newX, newY, 0), new Rotation3d(0, 0, tagAngle + rotation));

        return newPose;
      }

      /**
       * Return the desired pose of the robot.
       * If orientationOnly is true, this will return null.
       * @param ignoreForwards Whether to ignore the forwards distance.
       * @param ignoreSideways Whether to ignore the sideways distance.
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
      OFF(0, 0, 0),
      TARGET_FOUND(255, 188, 0), // orange
      AT_POSE(128, 255, 0),  // green
      WAITIING_FOR_CORAL(255, 248, 43),  // yellow
      CORAL_DETECTED(0, 157, 255);  // blue
   
      @Getter private final int red;
      @Getter private final int green;
      @Getter private final int blue;
  
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
      SCORE_L1(0.2, 3.0),  // TODO tune
      SCORE_L2(0.2, 3.0),  // TODO tune
      SCORE_L3(0.2, 3.0),  // TODO tune
      SCORE_L4(0.2, 3.0),  // TODO tune
      SCORE_BARGE(1.0, 2.0),  // TODO tune
      SCORE_PROCESSOR(1.0, 2.0),  // TODO tune
      INTAKE_REEF_ALGA(-0.3, 1.5),  // TODO tune
      INTAKE_CORAL(0.2, 5.0),  // Time is redundant; uses Canrange sensor  // TODO tune
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
