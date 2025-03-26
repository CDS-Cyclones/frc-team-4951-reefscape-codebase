// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStateConstants.CandleState;
import frc.robot.Constants.RobotStateConstants.IntakeAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Candle;

import java.util.function.Supplier;

public class IntakeActionCommand extends Command {
  private final Intake intake;
  private final Candle candle;
  private final Supplier<IntakeAction> intakeActionSupplier;
  private final boolean endless;
  private final Timer timer;
  
  /** 
   * A command that allows to control the intake.
   * 
   * @param intake The intake subsystem
   * @param intakeActionSupplier A supplier that supplies the intake action
   */
  public IntakeActionCommand(Intake intake, Candle candle, Supplier<IntakeAction> intakeActionSupplier, boolean endless) {
    this.intake = intake;
    this.candle = candle;
    this.intakeActionSupplier = intakeActionSupplier;
    this.endless = endless;
    this.timer = new Timer();
    
    addRequirements(this.intake, this.candle);
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
    intake.setSpeed(intakeActionSupplier.get().getSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    candle.setState(CandleState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!endless)
      return timer.hasElapsed(intakeActionSupplier.get().getTime());

    return false;
  }
}
