package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean[] motorConnected = {false, false};
    public double[] motorSpeed = {0.0, 0.0};
    public double[] motorCurrent = {0.0, 0.0};
    public double[] motorVoltage = {0.0, 0.0};
    public Warnings[] motorWarnings = {null, null};
    public Faults[] motorFaults = {null, null};
    public double[] motorTemperature = {0.0, 0.0};
    public double[] motorPosition = {0.0, 0.0};
    public double[] motorVelocity = {0.0, 0.0};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}
}