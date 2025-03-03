// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmOutCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final ProfiledPIDController pidController;


  /** Creates a new ArmOutCommand. */
  public ArmOutCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;

    pidController = new ProfiledPIDController(ArmConstants.armPID.kP, ArmConstants.armPID.kI, ArmConstants.armPID.kD, ArmConstants.armTrapezoidConstraints);

    addRequirements(this.armSubsystem);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setGoal(ArmConstants.ArmPosition.OUT.position);
    pidController.setTolerance(ArmConstants.kArmTolerance);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(armSubsystem.getArmPosition());
    armSubsystem.setArmSpeed(speed);
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
