// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateManager;
import lombok.Getter;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ManipulatorConstants.*;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements IntakeIO {
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  private final SparkMax intakeMotor = new SparkMax(intakeMotorId, MotorType.kBrushless);
  private final CANrange coralCanrange = new CANrange(coralCanrangeCanId);

  @Getter boolean coralDetected = false;

  public Intake() {
    intakeMotor.configure(intakeWheelsMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateSensorStatus();

    updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);

    try {
      String intakeStateString = "coral present: " + (coralDetected ? "yes" : "no");
      SmartDashboard.putString("Mutables/Intake State", intakeStateString);
      SmartDashboard.putString("Mutables/Intake Action", RobotStateManager.getDesiredIntakeAction().toString());
    } catch(Exception e) {}
  }

  /**
   * Sets the speed of the intake motor.
   *
   * @param speed The speed to set the motor to.
   */
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  /**
   *  Checks if Canrange detects a coral.
   *  If sensor is not connected, returns false.
   * 
   * @return True if a coral is detected.
   */
  private boolean detectCoral() {
    if (!coralCanrange.isConnected()) {
      return false;
    }

    return coralCanrange.getDistance().getValue().in(Meters) < coralCanrangeDistanceThreshold;
  }

  /**
   * Updates the intake state.
   */
  private void updateSensorStatus() {
    coralDetected = detectCoral();
  }

  /**
   * Stops the intake motor.
   */
  public void stop() {
    intakeMotor.stopMotor();
  }

  @Override
    public void updateInputs(IntakeIOInputs inputs) {
    inputs.connected = intakeMotor.getFirmwareVersion() != 0;
    inputs.intakeSpeed = intakeMotor.getAppliedOutput();
    inputs.intakeCurrent = intakeMotor.getOutputCurrent();
    inputs.intakeVoltage = intakeMotor.getBusVoltage();
    inputs.intakeTemperature = intakeMotor.getMotorTemperature();
    inputs.intakeAction = RobotStateManager.getDesiredIntakeAction();
    inputs.coralCanrangeConnected = coralCanrange.isConnected();
    inputs.coralCanrangeDistance = coralCanrange.getDistance().getValue().in(Meters);
    inputs.coralDetected = coralDetected;
  }
}
