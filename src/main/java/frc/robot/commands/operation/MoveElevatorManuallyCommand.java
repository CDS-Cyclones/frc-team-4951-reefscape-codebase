// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorManuallyCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final DoubleSupplier speedSupplier;


  /** Creates a new MoveElevatorManuallyCommand. */
  public MoveElevatorManuallyCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier speedSupplier) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.speedSupplier = speedSupplier;
  
    addRequirements(this.elevatorSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setElevatorSpeed(-speedSupplier.getAsDouble() * 0.2);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopElevator();
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
