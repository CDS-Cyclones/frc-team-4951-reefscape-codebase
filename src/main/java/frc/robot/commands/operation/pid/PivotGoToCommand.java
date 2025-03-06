// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Pivot;

import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotGoToCommand extends Command {
  private final Pivot pivot;
  private final DoubleSupplier desiredPositionSupplier;
  private final ProfiledPIDController pidController;

  /** Creates a new PivotGoToCommand. */
  public PivotGoToCommand(Pivot pivot, DoubleSupplier desiredPositionSupplier) {
    this.pivot = pivot;
    this.desiredPositionSupplier = desiredPositionSupplier;
    this.pidController = new ProfiledPIDController(
      Constants.ManipulatorConstants.kPivotPIDConstants.kP,
      Constants.ManipulatorConstants.kPivotPIDConstants.kI,
      Constants.ManipulatorConstants.kPivotPIDConstants.kD,
      Constants.ManipulatorConstants.kPivotTrapezoidConstraints
    );

    addRequirements(this.pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setGoal(desiredPositionSupplier.getAsDouble());
    pidController.setTolerance(Constants.ManipulatorConstants.kPivotTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pidController.getGoal().position != desiredPositionSupplier.getAsDouble()) {
      pidController.setGoal(desiredPositionSupplier.getAsDouble());
    }

    double speed = pidController.calculate(pivot.getPosition());
    double feedforward = pivot.calculateFeedforward(pidController.getSetpoint().velocity, 0);

    // pivot.setVoltage(speed);

    System.out.println("Pivot volt: " + desiredPositionSupplier.getAsDouble());


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
