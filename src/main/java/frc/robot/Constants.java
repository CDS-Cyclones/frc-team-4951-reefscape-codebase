// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);  // TODO switch to '25 for comp, using '24 b/c Web Components do not have '25 field.
    
    public static final PoseStrategy primaryMultiTagStrat = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public static final PoseStrategy fallbackSingleTagStrat = PoseStrategy.LOWEST_AMBIGUITY;

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);     // TODO tune
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);  // TODO tune

    public static final double VISION_YAW_DEADBAND = 1.5; // degrees // TODO tune

    public static final double VISION_TURN_kP = 0.00001; // TODO tune
  
    /**
     * Enum representing different vision pipelines.
     */
    public enum VisionPipelineInfo {
      TWO_D_APRIL_TAG_PIPELINE(0, "2d_apriltag_pipeline"),
      THREE_D_APRIL_TAG_PIPELINE(1, "3d_apriltag_pipeline");

      public final int pipelineIndex;
      public final String pipelineName;

      VisionPipelineInfo(int pipelineIndex, String pipelineName) {
        this.pipelineIndex = pipelineIndex;
        this.pipelineName = pipelineName;
      }
    }

    /**
     * Enum representing different vision cameras.
     */
    public static enum VisionCameraInfo {
      PRIMARY("cds_cam", new Transform3d());

      public final String camName;
      public final Transform3d botToCam;

      VisionCameraInfo(String camName, Transform3d botToCam) {
        this.camName = camName;
        this.botToCam = botToCam;
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
}