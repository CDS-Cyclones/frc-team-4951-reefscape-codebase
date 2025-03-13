package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.mutables.MutableCandleState.CandleState;

public interface CandleIO {
  @AutoLog
  public static class CandleIOInputs {
    public double temperature = 0.0;
    public double busVoltage = 0.0;
    public double railVoltage = 0.0;
    public double current = 0.0;
    public double brightness = 0.0;
    public CandleState state = CandleState.OFF;
  }

  public default void updateInputs(CandleIOInputs inputs) {}
}
