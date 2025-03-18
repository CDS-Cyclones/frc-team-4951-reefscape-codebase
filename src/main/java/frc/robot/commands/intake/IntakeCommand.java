// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.mutables.MutableIntakeAction.IntakeAction;
import frc.robot.subsystems.intake.Intake;

import java.util.function.Supplier;

public class IntakeCommand extends Command {
  private final Intake intake;
  private final Supplier<IntakeAction> intakeActionSupplier;
  private final Timer timer;
  
  /** 
   * A command that allows to control the intake.
   * 
   * @param intake The intake subsystem
   * @param intakeActionSupplier A supplier that supplies the intake action
   */
  public IntakeCommand(Intake intake, Supplier<IntakeAction> intakeActionSupplier) {
    this.intake = intake;
    this.intakeActionSupplier = intakeActionSupplier;
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
    intake.setSpeed(intakeActionSupplier.get().getSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intakeActionSupplier.get().isTimed())
      return timer.hasElapsed(intakeActionSupplier.get().getTime());
    else if (intakeActionSupplier.get().isConditional())
      return intakeActionSupplier.get().getEndState() == intake.getIntakeState();
    
    return false;
  }
}
