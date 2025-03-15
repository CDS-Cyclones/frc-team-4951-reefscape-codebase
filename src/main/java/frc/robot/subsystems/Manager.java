// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.mutables.MutableCandleState;
import frc.robot.mutables.MutableElevatorPosition;
import frc.robot.mutables.MutableIntakeAction;
import frc.robot.mutables.MutableIntakeState;
import frc.robot.mutables.MutablePivotPosition;

public class Manager extends SubsystemBase {
  /** Creates a new Manager. */
  public Manager() {}

  @Override
  public void periodic() {
    // Print values of all mutables to SmartDashboard
    try {
      SmartDashboard.putString("Mutables/Elevator Position", MutableElevatorPosition.getMutableElevatorPosition().toString());
      SmartDashboard.putString("Mutables/Pivot Position", MutablePivotPosition.getMutablePivotPosition().toString());
      SmartDashboard.putString("Mutables/Candle State", MutableCandleState.getMutableCandleState().toString());
      SmartDashboard.putString("Mutables/Intake Action", MutableIntakeAction.getMutableIntakeAction().toString());
      SmartDashboard.putString("Mutables/Intake State", MutableIntakeState.getMutableIntakeState().toString());
    } catch (Exception e) {}
  }
}
