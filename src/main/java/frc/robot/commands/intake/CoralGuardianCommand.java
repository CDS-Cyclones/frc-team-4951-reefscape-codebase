// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class CoralGuardianCommand extends Command {
  private final Intake intake;
  
  /** 
   * Repositions coral within intake in case of unintentional movement.
   * 
   * @param intake The intake subsystem
   */
  public CoralGuardianCommand(Intake intake) {
    this.intake = intake;
    
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!intake.isCoralDetectedAtOutflow()) {
      intake.setSpeed(0.15);
    } else if (!intake.isCoralDetectedAtInflow()) {
      intake.setSpeed(-0.15);
    } else {
      intake.stop();
    }
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
