// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.mutables.MutableElevatorPosition.ElevatorPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.mutables.MutableElevatorPosition;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.util.function.Supplier;

/**
 * A command that moves the elevator to a desired position using a {@link ProfiledPIDController} and keeps it there.
 * The position is determined in {@link MutableElevatorPosition}.
 */
public class ElevatorGoToCommand extends Command {
  private final Elevator elevator;
  private final Pivot pivot;
  private final Supplier<ElevatorPosition>  desiredPositionSupplier;
  private final ProfiledPIDController controller;

  /** Creates a new ElevatorGoToCommand. */
  public ElevatorGoToCommand(Elevator elevator, Pivot pivot, Supplier<ElevatorPosition> desiredPositionSupplier) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.desiredPositionSupplier = desiredPositionSupplier;
    
    controller = new ProfiledPIDController(elevatorKp, 0.0, elevatorKd, new TrapezoidProfile.Constraints(elevatorMaxSpeed, elevatorMaxAcceleration));

    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setGoal(desiredPositionSupplier.get().getPosition());
    controller.setTolerance(elevatorPIDTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getGoal().position != desiredPositionSupplier.get().getPosition()) {
      controller.setGoal(desiredPositionSupplier.get().getPosition());
    }

    // Calculate the speed of the elevator.
    double speed = controller.calculate(elevator.getPosition());
    speed += elevator.calculateFeedforward(controller.getSetpoint().velocity);

    // If the pivot is in the way of the elevator, prevent it from going up.
    if(!pivot.isOutOfElevatorWay() && speed > 0.0)
      speed = 0.0;

    // If the elevator is at the top or bottom, prevent it from moving further.
    if((elevator.getPosition() <= elevatorMinPosition && speed < 0.0) || (elevator.getPosition() >= elevatorMaxPosition && speed > 0.0))
      speed = 0.0;
    
    // Set the speed of the elevator.
    elevator.setSpeed(speed);
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
