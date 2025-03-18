// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateManager;
import frc.robot.Constants.RobotStateConstants.CandleState;
import frc.robot.Constants.RobotStateConstants.IntakeAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Candle;

public class IntakeCoral extends Command {
  private final Intake intake;
  private final Candle candle;
  
  /** 
   * Command to intake a coral.
   * Runs until the intake detects a coral.
   */
  public IntakeCoral(Intake intake, Candle candle) {
    this.intake = intake;
    this.candle = candle;
    
    addRequirements(this.intake, this.candle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    candle.setState(CandleState.WAITIING_FOR_CORAL);
    
    RobotStateManager.setIntakeOccupied(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set the speed of the intake
    intake.setSpeed(IntakeAction.INTAKE_CORAL.getSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    candle.setState(CandleState.OFF);
    RobotStateManager.setIntakeOccupied(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // intake.isCoralDetected();
  }
}
