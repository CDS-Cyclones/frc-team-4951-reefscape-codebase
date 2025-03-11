// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.elevator.Elevator;

public class HoldElevatorPositionCommand extends Command {
  private final Elevator elevator;

  /**
   * A command that uses a {@link ElevatorFeedforward} to keep the elevator at its current height.
   */
  public HoldElevatorPositionCommand(Elevator elevator) {
    this.elevator = elevator;
    
    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setSpeed(elevator.calculateFeedforward(0.0));
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
