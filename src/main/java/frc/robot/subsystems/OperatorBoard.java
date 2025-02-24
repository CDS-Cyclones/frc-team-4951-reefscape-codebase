// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.PoseRelToAprilTag;

public class OperatorBoard extends SubsystemBase {
  private final GenericHID operatorBoard;

  ///first sixteen buttons ar eeach related to specific pose in PosesRelToAprilTag express that relationshipo as map
  private final Map<Integer, PoseRelToAprilTag> buttonToPoseMap = Map.of(
    0, PoseRelToAprilTag.SAMPLE_POSE
    // TODO add the rest of the poses
  );


  /** Creates a new OperatorBoard. */
  public OperatorBoard(int port) {
    operatorBoard = new GenericHID(port);
  }


  /** This method will be called once per scheduler run. */
  @Override
  public void periodic() {
    // check if any buttons are pressed and if so, run the command associated with that button
    for (Map.Entry<Integer, PoseRelToAprilTag> entry : buttonToPoseMap.entrySet()) {
      if(operatorBoard.getRawButtonPressed(entry.getKey())) {
        // TODO
      }
    }
  }
}
