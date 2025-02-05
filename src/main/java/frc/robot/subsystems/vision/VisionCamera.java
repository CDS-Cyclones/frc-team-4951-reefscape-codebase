// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionCamera extends SubsystemBase {
  private final PhotonCamera               camera;


  /** Creates a new Camera. */
  public VisionCamera(String camName) {
    camera = new PhotonCamera(camName);
  }


  @Override
  public void periodic() {
  }


  public final Optional<PhotonPipelineResult> getLatestResult() {
    List<PhotonPipelineResult> allUnreadResults = camera.getAllUnreadResults();

    return allUnreadResults.isEmpty() ? Optional.empty() : Optional.of(allUnreadResults.get(allUnreadResults.size() - 1));
  }


}
