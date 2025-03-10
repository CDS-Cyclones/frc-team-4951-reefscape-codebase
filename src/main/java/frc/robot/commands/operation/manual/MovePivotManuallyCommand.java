// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.manual;

import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.pivot.Pivot;
import frc.robot.mutables.MutableElevatorPosition;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.util.function.DoubleSupplier;

/**
 * A command that moves the elevator to a desired position using a {@link ProfiledPIDController} and keeps it there.
 * The position is determined in {@link MutableElevatorPosition}.
 */
public class MovePivotManuallyCommand extends Command {
  private final Pivot pivot;
  private final DoubleSupplier speedSupplier;

  /** Creates a new ElevatorGoToCommand. */
  public MovePivotManuallyCommand(Pivot pivot, DoubleSupplier speedSupplier) {
    this.pivot = pivot;
    this.speedSupplier = speedSupplier;
    
    addRequirements(this.pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedSupplier.getAsDouble();
    boolean safeToRun = false;

    // If the pivot is fully in or out, prevent it from moving further.
    if((pivot.getPosition() <= pivotMinPosition && speed < 0.0) || (pivot.getPosition() >= pivotMaxPosition && speed > 0.0))
      safeToRun = false;
    
    // Run elevator if it is safe to do so.
    if(safeToRun)
      pivot.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
