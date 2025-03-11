// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.pivot.Pivot;

public class HoldPivotPositionCommand extends Command {
  private final Pivot pivot;

  /**
   * A command that uses a {@link ArmFeedforward} to keep the pivot at its current position.
   */
  public HoldPivotPositionCommand(Pivot pivot) {
    this.pivot = pivot;

    addRequirements(this.pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setSpeed(pivot.calculateFeedforward(pivot.getPosition(),0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
