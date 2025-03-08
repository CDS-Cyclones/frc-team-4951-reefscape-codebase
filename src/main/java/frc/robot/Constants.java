// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;  // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27);  // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(32.5);  // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;  // Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;            // Math.PI;
    public static final double kBackLeftChassisAngularOffset = Math.PI;;       // 0;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;   // -Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 21;
    public static final int kRearLeftDrivingCanId = 27;
    public static final int kFrontRightDrivingCanId = 23;
    public static final int kRearRightDrivingCanId = 25;

    public static final int kFrontLeftTurningCanId = 22;
    public static final int kRearLeftTurningCanId = 28;
    public static final int kFrontRightTurningCanId = 24;
    public static final int kRearRightTurningCanId = 26;

    public static final String kGyroCanBusName = "pigeonbus";
    public static final int kGyroCanId = 38;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear,
    // 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularSpeedRadiansPerSecondSquared
    );
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
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

    public static final PoseStrategy primaryMultiTagStrat = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public static final PoseStrategy fallbackSingleTagStrat = PoseStrategy.LOWEST_AMBIGUITY;

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
     * Enum representing different vision cameras.
     */
    public static enum VisionCameraInfo {
      PRIMARY(
        "cds_cam",
        new Transform3d(
          new Translation3d(0.406, 0, 0.1524), // X is forward in m, z is up in m
          new Rotation3d(0, 0, 0)  // facing forward
        )
      );

      public final String camName;
      public final Transform3d botToCam;

      VisionCameraInfo(String camName, Transform3d botToCam) {
        this.camName = camName;
        this.botToCam = botToCam;
      }
    }


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
