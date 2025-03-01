// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final ElevatorPosition desiredPosition;
  private final ProfiledPIDController pidController;


  /** Creates a new ElevatorGoToCommand. */
  public ElevatorGoToCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPosition desiredPosition) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.desiredPosition = desiredPosition;

    pidController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2, 1));

    addRequirements(this.elevatorSubsystem);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setGoal(desiredPosition.position);
    pidController.setTolerance(0.5);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(elevatorSubsystem.getElevatorPosition());
    elevatorSubsystem.setElevatorSpeed(speed);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atGoal();
  }
}
