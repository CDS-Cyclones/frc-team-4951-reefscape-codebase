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
import frc.robot.Constants.RobotStateConstants.IntakeState;
import lombok.Getter;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ManipulatorConstants.*;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements IntakeIO {
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  private final SparkMax intakeMotor = new SparkMax(intakeMotorId, MotorType.kBrushless);
  private final CANrange rangeSensor = new CANrange(canrangeCanId, canrangeCanBus);
  
  @Getter private IntakeState intakeState;

  public Intake() {
    intakeMotor.configure(intakeWheelsMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    updateIntakeState();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateIntakeState();

    updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);

    try {
      SmartDashboard.putString("Mutables/Intake State", intakeState.toString());
    } catch (Exception e) {}
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
   * Updates the intake state based on the range sensor.
   */
  private void updateIntakeState() {
    if (!rangeSensor.isConnected()) {
      intakeState = IntakeState.UNKNOWN;
      return;
    }
    
    double sensorValue = rangeSensor.getDistance().getValue().in(Meters);
    
    if (sensorValue < intakeRangeForCoral) {
      intakeState = IntakeState.CORAL;
    } else if (sensorValue < intakeRangeForAlga) {
      intakeState = IntakeState.ALGA;
    } else {
      intakeState = IntakeState.EMPTY;
    }
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
    inputs.intakeDistance = rangeSensor.getDistance().getValue().in(Meters);
    inputs.intakeState = intakeState;
  }
}
