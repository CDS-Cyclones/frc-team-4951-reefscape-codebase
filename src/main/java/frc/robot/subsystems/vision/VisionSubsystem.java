// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.VisionCameraInfo;


/**
 * A subsystem that interfaces with the PhotonVision camera and processes vision data to estimate the
 * robot's pose on the field.
 */
public class VisionSubsystem extends SubsystemBase {
  public final List<PVCamera> cameras = new ArrayList<>();


  /** Creates a new Camera. */
  public VisionSubsystem() {
    for (VisionCameraInfo camInfo : VisionCameraInfo.values()) {
      cameras.add(new PVCamera(camInfo.camName, camInfo.botToCam));
    }
  }


  /**
   * Returns a list of all cameras.
   *
   * @return A {@link List} containing all {@link PVCamera}s.
   */
  public List<PVCamera> getCameras() {
    return cameras;
  }
}
