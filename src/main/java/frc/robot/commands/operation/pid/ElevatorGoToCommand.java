// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.DesiredFieldPose;
import frc.robot.subsystems.manipulator.Elevator;

import java.util.function.DoubleSupplier;

/**
 * A command that moves the elevator to a desired position using a {@link ProfiledPIDController} and keeps it there.
 * The position is determined in {@link DesiredFieldPose}.
 */
public class ElevatorGoToCommand extends Command {
  private final Elevator elevator;
  private final DoubleSupplier desiredPositionSupplier;
  private final ProfiledPIDController pidController;

  /** Creates a new ElevatorGoToCommand. */
  public ElevatorGoToCommand(Elevator elevator, DoubleSupplier desiredPositionSupplier) {
    this.elevator = elevator;
    this.desiredPositionSupplier = desiredPositionSupplier;
    this.pidController = new ProfiledPIDController(
      Constants.ManipulatorConstants.kElevatorPIDConstants.kP,
      Constants.ManipulatorConstants.kElevatorPIDConstants.kI,
      Constants.ManipulatorConstants.kElevatorPIDConstants.kD,
      Constants.ManipulatorConstants.kElevatorTrapezoidConstraints
    );

    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setGoal(desiredPositionSupplier.getAsDouble());
    pidController.setTolerance(Constants.ManipulatorConstants.kElevatorTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pidController.getGoal().position != desiredPositionSupplier.getAsDouble()) {
      pidController.setGoal(desiredPositionSupplier.getAsDouble());
    }
    double feedforward = elevator.calculateFeedforward(pidController.getSetpoint().velocity);

    double speed = pidController.calculate(elevator.getPosition());
    speed += feedforward;
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
