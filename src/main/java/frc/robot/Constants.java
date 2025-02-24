// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveConstants {
    /** Maximum speed of the robot in meters per second, used to limit acceleration. */
    public static final double MAX_SPEED  = Units.feetToMeters(4.5);

    /** High verbosity allows to see more data in ShuffleBoard */
    public static final TelemetryVerbosity TELEMETRY_VERBOSITY = TelemetryVerbosity.HIGH;
  }


  /**
   * VisionConstants class containing enums for Vision Pipelines and Vision Cameras.
   */
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
     * Enum representing different vision pipelines.
     */
    public enum VisionPipelineInfo {
      TWO_D_APRIL_TAG_PIPELINE(1, "2d_apriltag_pipeline"),
      THREE_D_APRIL_TAG_PIPELINE(0, "3d_apriltag_pipeline");

      public final int pipelineIndex;
      public final String pipelineName;

      VisionPipelineInfo(int pipelineIndex, String pipelineName) {
        this.pipelineIndex = pipelineIndex;
        this.pipelineName = pipelineName;
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


  public static class DriverJoystickConstants {
    public static final int kDriverControllerPort = 0;

    public static final double kTurnMultiplier = 0.8;

    // Joystick deadbands for driving
    public static final double kLeftXDeadband  = 0.1;
    public static final double kLeftYDeadband  = 0.1;
    public static final double kRightXDeadband = 0.1;
  }


  public static class OperatorBoardConstants {
    public static final int kOperatorBoardPort = 1;
  }
}
