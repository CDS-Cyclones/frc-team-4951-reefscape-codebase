// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operation.manual;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArmManuallyCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final DoubleSupplier speedSupplier;


  /** Creates a new MoveArmManuallyCommand. */
  public MoveArmManuallyCommand(ArmSubsystem armSubsystem, DoubleSupplier speedSupplier) {
    this.armSubsystem = armSubsystem;
    this.speedSupplier = speedSupplier;
  
    addRequirements(this.armSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!(armSubsystem.getArmPosition() <= ArmConstants.kArmMin && armSubsystem.getArmPosition() >= ArmConstants.kArmMax)) {
      armSubsystem.setArmSpeed(-speedSupplier.getAsDouble());
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopArm();
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
