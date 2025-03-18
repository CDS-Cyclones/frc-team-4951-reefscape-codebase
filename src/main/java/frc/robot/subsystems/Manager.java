// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateManager;

public class Manager extends SubsystemBase {
  /** Creates a new Manager. */
  public Manager() {}

  @Override
  public void periodic() {
    // Print values of all mutables to SmartDashboard
    try {
      SmartDashboard.putString("Mutables/Elevator Position", RobotStateManager.getDesiredElevatorPosition().toString());
      SmartDashboard.putString("Mutables/Pivot Position", RobotStateManager.getDesiredPivotPosition().toString());
    //   SmartDashboard.putString("Mutables/Candle State", MutableCandleState.getMutableCandleState().toString());
    //   SmartDashboard.putString("Mutables/Intake Action", MutableIntakeAction.getMutableIntakeAction().toString());
    //   SmartDashboard.putString("Mutables/Intake State", MutableIntakeState.getMutableIntakeState().toString());
      SmartDashboard.putString("Mutables/Field Pose", RobotStateManager.getDesiredFieldPose().toString());
    } catch (Exception e) {}
  }
}
