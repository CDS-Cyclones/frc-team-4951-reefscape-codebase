// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.PVCamera;
import frc.robot.subsystems.vision.VisionSubsystem;


/** A subsystem that estimates the robot's pose on the field using odometry and vision data. */
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
    // Update the odometry of the swerve drive
    swerve.updateOdometry();

    // Add vision measurements from all cameras to the swerve drive
    for (PVCamera camera : vision.getCameras())
    {
      Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
      if (poseEst.isPresent())
      {
        var pose = poseEst.get();
        swerve.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, camera.getEstimationStdDevs());
      }
    }
  }
}
