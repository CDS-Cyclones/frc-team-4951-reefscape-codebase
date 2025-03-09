// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.manual;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MovePivotManuallyCommand extends Command {
  private final Pivot pivotSubsystem;
  private final boolean moveOut;

  /** Creates a new MoveArmManuallyCommand. */
  public MovePivotManuallyCommand(Pivot pivotSubsystem, boolean moveOut) {
    this.pivotSubsystem = pivotSubsystem;
    this.moveOut = moveOut;

    addRequirements(this.pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//    if(!(pivotSubsystem.getArmPosition() <= ArmConstants.kArmMin && pivotSubsystem.getArmPosition() >= ArmConstants.kArmMax)) {
//      pivotSubsystem.setArmSpeed(-speedSupplier.getAsDouble());
//    }
    if (moveOut) {
      pivotSubsystem.setSpeed(-0.2);
    } else {
      pivotSubsystem.setSpeed(0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
