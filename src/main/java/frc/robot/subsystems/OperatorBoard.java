// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.PoseRelToAprilTag;

public class OperatorBoard extends SubsystemBase {
  private final GenericHID operatorBoard;

  private int desiredRobotPoseID = -1;

  private boolean[] holdButtonsStates = {false, false, false, false, false, false};


  /** Creates a new OperatorBoard. */
  public OperatorBoard(int port) {
    operatorBoard = new GenericHID(port);
  }


  /** This method will be called once per scheduler run. */
  @Override
  public void periodic() {
    // Coral poses buttons
    for(int i=1; i<13; i++) {
      if(operatorBoard.getRawButtonPressed(i)) {
        desiredRobotPoseID = i - 1;
        System.out.println("Desired Robot Pose ID: " + desiredRobotPoseID);
      }
    }


    // // // Hold buttons
    // for(int i=13; i<19; i++) {
    //   holdButtonsStates[i-13] = operatorBoard.getRawButtonPressed(i);
    // }
    // // print IDS  of the hodl buttons are cufrrntly presed
    // for(int i=0; i<6; i++) {
    //   if(holdButtonsStates[i]) {
    //     System.out.println("Hold Button " + i + " is pressed");
    //   }
    // }


  }
}
