// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ManipulatorConstants.*;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements IntakeIO {
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  private final SparkMax intakeMotor = new SparkMax(intakeMotorId, MotorType.kBrushless);
  private final CANrange rangeSensor = new CANrange(canrangeCanId, canrangeCanBus);

  public Intake() {
    intakeMotor.configure(intakeWheelsMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
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
   * Returns whether the coral is in the intake.
   *
   * @return Whether the coral is in the intake.
   */
  public boolean isCoralInIntake() {
    return rangeSensor.getDistance().getValue().in(Meters) < intakeRangeSensorThreshold;
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
    inputs.canrangeConnected = rangeSensor.isConnected();
    inputs.coralInIntake = isCoralInIntake();
    inputs.intakeDistance = rangeSensor.getDistance().getValue().in(Meters);
  }
}
