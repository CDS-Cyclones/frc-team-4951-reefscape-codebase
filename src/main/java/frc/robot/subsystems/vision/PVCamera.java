// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;


/**
 * A subsystem that interfaces with the PhotonVision camera and processes vision data to estimate the
 * robot's pose on the field.
 */
public class PVCamera extends SubsystemBase {
  public final PhotonCamera camera;
  private final Transform3d botToCam;
  private final PhotonPoseEstimator photonPoseEstimator;

  private List<PhotonPipelineResult> allUnreadResults3D;
  private Matrix<N3, N1> curStdDevs;


  /** Creates a new Camera. */
  public PVCamera(String camName, Transform3d botToCam) {
    this.camera = new PhotonCamera(camName);
    this.botToCam = botToCam;
    this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, primaryMultiTagStrat, botToCam);
    this.photonPoseEstimator.setMultiTagFallbackStrategy(fallbackSingleTagStrat);

    updateResults(); // Fetch initial results
  }


  /** This method will be called once per scheduler run. */
  @Override
  public void periodic() {
    updateResults();
  }


  /**
   * Updates {@link allUnreadResults2D} and {@link allUnreadResults3D} for both 2D and 3D pipelines.
   */
  public void updateResults() {
    camera.setPipelineIndex(VisionPipelineInfo.THREE_D_APRIL_TAG_PIPELINE.pipelineIndex);
    allUnreadResults3D = camera.getAllUnreadResults(); 
  }


  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   * used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : allUnreadResults3D) {
      if(change.hasTargets()) {
        visionEst = photonPoseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
     
      }
    }
    return visionEst;
  }


  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame.
   */
  private void updateEstimationStdDevs(
    Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30)); curStdDevs = estStdDevs;
      }
    }
  }


  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }


  /**
   * Retrieves the latest result from the camera 3D pipeline.
   * 
   * <p>This method fetches all unread results from #D april ttag pipeline and returns the latest one if available.
   * If no unread results are found, it returns an empty {@link Optional}.
   *
   * @return an {@link Optional} containing the latest {@link PhotonPipelineResult} if available,
   * or an empty {@link Optional} if there are no unread results.
   */
  public Optional<PhotonPipelineResult> getLatestResult() {
    return allUnreadResults3D.isEmpty() ? Optional.empty() : Optional.of(allUnreadResults3D.get(allUnreadResults3D.size() - 1));
  }


  /**
   * Returns the transform from the robot's center of rotation to the camera.
   * 
   * @return {@link Transform3D} from the robot's center of rotation to the camera.
   * 
   */
  public Transform3d getBotToCam() {
    return botToCam;
  }
}
