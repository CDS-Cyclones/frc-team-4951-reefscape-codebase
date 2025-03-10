package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double intakeSpeed = 0.0;
    public double intakeCurrent = 0.0;
    public double intakeVoltage = 0.0;
    public Warnings intakeWarnings = null;
    public Faults intakeFaults = null;
    public double intakeTemperature = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}