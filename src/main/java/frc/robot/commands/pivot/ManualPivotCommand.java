// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.pivot.Pivot;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.util.function.DoubleSupplier;

public class ManualPivotCommand extends Command {
  private final Pivot pivot;
  private final DoubleSupplier speedSupplier;

  /**
   * A command that allows to manually control the pivot.
   */
  public ManualPivotCommand(Pivot pivot, DoubleSupplier speedSupplier) {
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
