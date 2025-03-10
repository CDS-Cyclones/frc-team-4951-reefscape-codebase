// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.mutables.MutablePivotPosition.PivotPosition;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.mutables.MutablePivotPosition;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.util.function.Supplier;

/**
 * A command that moves the elevator to a desired position using a {@link ProfiledPIDController} and keeps it there.
 * The position is determined in {@link MutablePivotPosition}.
 */
public class PivotGoToCommand extends Command {
  private final Pivot pivot;
  private final Supplier<PivotPosition>  desiredPositionSupplier;
  private final ProfiledPIDController controller;

  private double lastSpeed;
  private double lastTime;

  /** Creates a new ElevatorGoToCommand. */
  public PivotGoToCommand(Pivot pivot, Supplier<PivotPosition> desiredPositionSupplier) {
    this.pivot = pivot;
    this.desiredPositionSupplier = desiredPositionSupplier;
    
    controller = new ProfiledPIDController(pivotKp, 0.0, pivotKd, new TrapezoidProfile.Constraints(pivotMaxSpeed, pivotMaxAcceleration));

    addRequirements(this.pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setGoal(desiredPositionSupplier.get().getPosition());
    controller.setTolerance(pivotPIDTolerance);

    lastSpeed = 0.0;
    lastTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getGoal().position != desiredPositionSupplier.get().getPosition()) {
      controller.setGoal(desiredPositionSupplier.get().getPosition());
    }

    // Calculate the speed of the pivot
    double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    double speed = controller.calculate(pivot.getPosition());
    speed += pivot.calculateFeedforward(controller.getSetpoint().velocity, acceleration);

    // If the pivot is fully in or out, prevent it from moving further.
    if((pivot.getPosition() <= pivotMinPosition && speed < 0.0) || (pivot.getPosition() >= pivotMaxPosition && speed > 0.0))
      speed = 0.0;
    
    // Set the speed of the pivot
    pivot.setSpeed(speed);

    // Update tracking variables
    lastSpeed = controller.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
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
