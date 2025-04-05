package frc.robot.subsystems.wolverine;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface WolverineIO {
  @AutoLog
  public static class WolverineIOInputs {
    public Value value = Value.kOff;
  }

  public default void updateInputs(WolverineIOInputs inputs) {}
}
