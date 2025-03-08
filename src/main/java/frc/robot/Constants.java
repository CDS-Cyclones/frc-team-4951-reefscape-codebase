// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

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

  public class DriveConstants {
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
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
    public static final double driveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 40;
    public static final double turnMotorReduction = 4.71 / 1;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 52;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
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
  }

  public static final class OIConstants {
    // Joystick ports
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kOperatorBoardPort = 2;
    public static final int kOperatorControllerManualBackupPort = 3;

    // Joystick axis deadband
    public static final double kJoystickAxisDeadband = 0.1;
  }

  public static final class LimelightConstants {
    public static final String kLimelightName = "limelight";
  }

  public static final class ManipulatorConstants {
    public static final int kElevatorMotor1Id = 31;
    public static final int kElevatorMotor2Id = 32;
    public static final int kPivotMotorId = 56;
    public static final int kIntakeWheelsMotorId = 57;

    public static final SparkBaseConfig kElevatorMotor1Config = new SparkMaxConfig()
      .smartCurrentLimit(80)
      .secondaryCurrentLimit(90)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(false);
    public static final SparkBaseConfig kElevatorMotor2Config = new SparkMaxConfig()
      .smartCurrentLimit(80)
      .secondaryCurrentLimit(90)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(false);
    public static final SparkBaseConfig kPivotMotorConfig = new SparkMaxConfig()
      .smartCurrentLimit(80)
      .secondaryCurrentLimit(90)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(true);
    public static final SparkBaseConfig kIntakeWheelsMotorConfig = new SparkMaxConfig()
      .smartCurrentLimit(80)
      .secondaryCurrentLimit(90)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(false);

    public static final double kPivotMinPositionForElevatorMovement = 10000000;  // TODO figure this out

    public static final PIDConstants kElevatorPIDConstants = new PIDConstants(1, 0, 0);
    public static final TrapezoidProfile.Constraints kElevatorTrapezoidConstraints = new TrapezoidProfile.Constraints(1, 0.5);
    public static final double kElevatorTolerance = 0.05;
    public static final double kElevatorFeedforwardVelocity = 0.05;

    public static final PIDConstants kPivotPIDConstants = new PIDConstants(3, 0, 0);
    public static final TrapezoidProfile.Constraints kPivotTrapezoidConstraints = new TrapezoidProfile.Constraints(5, 3);
    public static final double kPivotTolerance = 0.05;
    public static final double kPivotFeedforwardVelocityOut = -0.05;
    public static final double kPivotFeedforwardVelocityIn = 0.3;















    public static final double kArmMin = 0.086;
    public static final double kArmMax = 0.586;

    public static final TrapezoidProfile.Constraints elevatorTrapezoidConstraints = new TrapezoidProfile.Constraints(1.5, 1);
    public static final PIDConstants elevatorPID = new PIDConstants(1, 0, 0);
    public static final double kArmTolerance = 0.5;
  }












  public static final class VisionConstants {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);  // TODO switch to '25 for comp, using '24 b/c Web Components do not have '25 field.

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);     // TODO tune
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);  // TODO tune

    public static final double VISION_YAW_DEADBAND = .5;  // TODO tune
    public static final double AMBIGUITY_DEADBAND = 0.2;

    public static final double VISION_TURN_kP = 0.005;    // TODO tune
    public static final double VISION_FORWARD_kP = 1;     // TODO tune

    // Constraints for profiled movement of robot whilst controlled by vision
    public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    // PID Values for ProfiledPIDControllers used in ChaseTagCommand
    public static final PIDConstants X_PID_CONSTANTS = new PIDConstants(0.8, 0, 0.02);       // TODO tune
    public static final PIDConstants Y_PID_CONSTANTS = new PIDConstants(0.4, 0, 0.07);       // TODO tune
    public static final PIDConstants OMEGA_PID_CONSTANTS = new PIDConstants(1, 0, 0);        // TODO tune

    // ProfiledPIDControllers tolerance values
    public static final double X_TOLERANCE = 0.2; // in m
    public static final double Y_TOLERANCE = 0.2; // in m
    public static final double OMEGA_TOLERANCE = 3; // in deg

    /**
     * Enum representing different desired poses relative to AprilTags on the field.
     */
    public static enum PoseRelToAprilTag {
      SAMPLE_POSE(18, 1.5, 0);

      /** ID of the april tag */
      public final int aprilTagId;

      /** Where the robot should be in relation to the tag */
      public final Transform3d relativePose;

      PoseRelToAprilTag(int aprilTagId, double metersInFront, double metersToTheLeft) {
        this.aprilTagId = aprilTagId;
        this.relativePose = new Transform3d(
          new Translation3d(metersInFront, metersToTheLeft, 0),
          new Rotation3d(0, 0, Math.PI) // PI rads (180 deg) means facing the tag
        );
      }
    }
  }


  public static class CANdleConstants {
    public static final int CANid = 40;
    public static final String CANbus = "pigeonbus";
  }

















  public static class IntakeConstants {
    public static final int kIntakeWheelsMotorPort = 57;

    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final boolean kInverted = false;

    public static final double kAlgaIntakeSpeed = -0.5;
    public static final double kAlgaOuttakeSpeed = 1;
    public static final double kCoralOuttakeSpeed = 0.5;

    public static final double kAlgaIntakeTime = 1;
    public static final double kAlgaOuttakeTime = 1;
    public static final double kCoralIntakeTime = 1;
    public static final double kCoralOuttakeTime = 1;
  }
}
