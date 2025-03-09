// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.manipulator.IntakeWheels;

public class MoveIntakeWheelsManuallyCommand extends Command {
  private final IntakeWheels intakeSubsystem;
  private final double speed;

  /** Creates a new OuttakeCoralManuallyCommand. */
  public MoveIntakeWheelsManuallyCommand(IntakeWheels intakeSubsystem, double speed) {
    this.intakeSubsystem = intakeSubsystem;
    this.speed = speed;

    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
