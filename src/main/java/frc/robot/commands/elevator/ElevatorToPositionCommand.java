// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.mutables.MutableElevatorPosition.ElevatorPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import static frc.robot.Constants.ManipulatorConstants.*;

public class ElevatorToPositionCommand extends Command {
  private final Elevator elevator;
  private final Pivot pivot;
  private final ElevatorPosition desiredPosition;
  private final ProfiledPIDController controller;

  /**
   * A command that moves the elevator to a desired position using a {@link ProfiledPIDController}.
   * The position is determined in {@link MutableElevatorPosition}.
   */
  public ElevatorToPositionCommand(Elevator elevator, Pivot pivot, ElevatorPosition desiredPosition) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.desiredPosition = desiredPosition;
    
    controller = new ProfiledPIDController(elevatorKp, 0.0, elevatorKd, new TrapezoidProfile.Constraints(elevatorMaxSpeed, elevatorMaxAcceleration));

    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setGoal(desiredPosition.getPosition());
    controller.setTolerance(elevatorPIDTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atGoal();
  }
}
