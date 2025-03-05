// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Positions;
import frc.robot.subsystems.manipulator.Elevator;
import frc.robot.subsystems.manipulator.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToCommand extends Command {
  private final Elevator elevator;
  private final Positions.ElevatorPosition desiredPosition;
  private final ProfiledPIDController pidController;

  /** Creates a new ElevatorGoToCommand. */
  public ElevatorGoToCommand(Elevator elevator, Positions.ElevatorPosition desiredPosition) {
    this.elevator = elevator;
    this.desiredPosition = desiredPosition;
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
    pidController.setGoal(desiredPosition.getPosition());
    pidController.setTolerance(Constants.ManipulatorConstants.kPivotTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(elevator.getPosition());
    elevator.setSpeed(speed);
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
