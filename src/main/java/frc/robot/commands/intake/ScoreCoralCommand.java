// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;
import static frc.robot.Constants.ManipulatorConstants.*;

public class ScoreCoralCommand extends Command {
  private final Intake intake;
  private final Timer timer;

  /** 
   * Outtake coral for a set amount of time.
   */
  public ScoreCoralCommand(Intake intake) {
    this.intake = intake;
    this.timer = new Timer();
    
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set the speed of the intake
    intake.setSpeed(coralIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(coralScoringTime);
  }
}
