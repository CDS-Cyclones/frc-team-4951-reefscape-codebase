// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimatorSubsystem extends SubsystemBase {
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;


  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimatorSubsystem(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;


  }


  @Override
  public void periodic() {
    swerve.updateOdometry();
    Optional<EstimatedRobotPose> poseEst = vision.getEstimatedGlobalPose();
    if (poseEst.isPresent())
    {
      var pose = poseEst.get();
      swerve.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, vision.getEstimationStdDevs());
    }
  }
}
