// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.manual;

import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;
import frc.robot.mutables.MutableElevatorPosition;

import java.util.function.DoubleSupplier;

/**
 * A command that moves the elevator to a desired position using a {@link ProfiledPIDController} and keeps it there.
 * The position is determined in {@link MutableElevatorPosition}.
 */
public class MoveIntakeManuallyCommand extends Command {
  private final Intake intake;
  private final DoubleSupplier speedSupplier;

  /** Creates a new ElevatorGoToCommand. */
  public MoveIntakeManuallyCommand(Intake intake, DoubleSupplier speedSupplier) {
    this.intake = intake;
    this.speedSupplier = speedSupplier;
    
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedSupplier.getAsDouble();

    // Set the speed of the intake
    intake.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
