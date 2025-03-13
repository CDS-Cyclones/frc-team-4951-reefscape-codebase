// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.pivot.Pivot;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.util.function.DoubleSupplier;

public class PivotToPositionCommand extends Command {
  private final Pivot pivot;
  private final DoubleSupplier desiredPositionSupplier;
  private ProfiledPIDController controller;

  /**
   * A command that moves the elevator to a desired position using a {@link ProfiledPIDController} and keeps it there.
   */
  public PivotToPositionCommand(Pivot pivot, DoubleSupplier desiredPositionSupplier) {
    this.pivot = pivot;
    this.desiredPositionSupplier = desiredPositionSupplier;
    
    addRequirements(this.pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = new ProfiledPIDController(
      pivotKp.getAsDouble(),
      0.0,
      pivotKd.getAsDouble(),
      new TrapezoidProfile.Constraints(
        pivotMaxSpeed.getAsDouble(),
        pivotMaxAcceleration.getAsDouble()
      )
    );

    controller.setGoal(desiredPositionSupplier.getAsDouble());
    controller.setTolerance(pivotPIDTolerance.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate the speed of the pivot
    double speed = controller.calculate(pivot.getPosition());
    speed += pivot.calculateFeedforward(pivot.getPosition(), controller.getSetpoint().velocity);

    // If the pivot is fully in or out, prevent it from moving further.
    if((pivot.getPosition() <= pivotMinPosition && speed < 0.0) || (pivot.getPosition() >= pivotMaxPosition && speed > 0.0))
      speed = 0.0;
     
    // Set the speed of the pivot
    pivot.setSpeed(speed);
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
