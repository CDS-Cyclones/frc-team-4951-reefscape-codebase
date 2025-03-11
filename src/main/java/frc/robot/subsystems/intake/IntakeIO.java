package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double intakeSpeed = 0.0;
    public double intakeCurrent = 0.0;
    public double intakeVoltage = 0.0;
    public double intakeTemperature = 0.0;

    public boolean canrangeConnected = false;
    public boolean coralInIntake = false;
    public double intakeDistance = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}