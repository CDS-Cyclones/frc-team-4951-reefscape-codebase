package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.RobotStateConstants.IntakeAction;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double intakeSpeed = 0.0;
    public double intakeCurrent = 0.0;
    public double intakeVoltage = 0.0;
    public double intakeTemperature = 0.0;
    public IntakeAction intakeAction = IntakeAction.NONE;
    public boolean coralCanrangeConnected = false;
    public double coralCanrangeDistance = 0.0;
    public boolean coralDetected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}