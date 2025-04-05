// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wolverine;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pivot.Pivot;
import lombok.Getter;

import org.littletonrobotics.junction.Logger;

// in honour of our Windsor teammates
public class Wolverine extends SubsystemBase implements WolverineIO {
  private final DoubleSolenoid solenoid = new DoubleSolenoid(61, PneumaticsModuleType.REVPH, 0, 1);
  private final WolverineIOInputsAutoLogged wolverineInputs = new WolverineIOInputsAutoLogged();

  @Getter private Value value = Value.kReverse;

  public Wolverine() {
  }

  @Override
  public void periodic() {
    updateInputs(wolverineInputs);
    Logger.processInputs("Wolverine", wolverineInputs);

    try {
      SmartDashboard.putString("Wolverine/Value", value.toString());
    } catch(Exception e) {}
  }

  private void setValue(Value value) {
    this.value = value;
    solenoid.set(value);
  }

  public void invertValue() {
    if (value == Value.kForward) {
      setValue(Value.kReverse);
    } else {
      setValue(Value.kForward);
    }
  }

  public Command invertWolverine(Pivot pivot) {
    return Commands.runOnce(
      this::invertValue
    ).onlyIf(
        pivot::isOutOfElevatorWay
    );
  }

  @Override
  public void updateInputs(WolverineIOInputs inputs) {
    inputs.value = getValue();
  }
}