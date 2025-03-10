// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.manual;

import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.mutables.MutableElevatorPosition;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.util.function.DoubleSupplier;

/**
 * A command that moves the elevator to a desired position using a {@link ProfiledPIDController} and keeps it there.
 * The position is determined in {@link MutableElevatorPosition}.
 */
public class MoveElevatorManuallyCommand extends Command {
  private final Elevator elevator;
  private final Pivot pivot;
  private final DoubleSupplier speedSupplier;

  /** Creates a new ElevatorGoToCommand. */
  public MoveElevatorManuallyCommand(Elevator elevator, Pivot pivot, DoubleSupplier speedSupplier) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.speedSupplier = speedSupplier;
    
    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedSupplier.getAsDouble();
    boolean safeToRun = true;

    // If the pivot is in the way of the elevator, prevent it from going up.
    if(!pivot.isOutOfElevatorWay() && speed > 0.0)
      safeToRun = false;

    // If the elevator is at the top or bottom, prevent it from moving further.
    if((elevator.getPosition() <= elevatorMinPosition && speed < 0.0) || (elevator.getPosition() >= elevatorMaxPosition && speed > 0.0))
      safeToRun = false;
    
    // Run elevator if it is safe to do so.
    if(safeToRun)
      elevator.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
