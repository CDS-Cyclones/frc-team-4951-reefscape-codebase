// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
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
  public static final SparkBaseConfig intakeMotorConfig = new SparkMaxConfig();
  private final CANrange coralStartCanrange = new CANrange(coralStartCanrangeCanId, coralStartCanrangeCanBus);
  private final CANrange coralCompleteCanrange = new CANrange(coralCompleteCanrangeCanId, coralCompleteCanrangeCanBus);

  @Getter boolean coralDetected = false;
  @Getter boolean coralCompletelyIn = false;

  public Intake() {
    intakeMotorConfig
    .smartCurrentLimit(80)
    .secondaryCurrentLimit(90)
    .idleMode(SparkBaseConfig.IdleMode.kBrake)
    .inverted(intakeMotorInverted);
    intakeMotor.configure(intakeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateSensorStatus();

    updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);

    try {
      // modidfy the string to say if its in or fully in
      // pirnt values from canranges DISTANCES IN METERS
      SmartDashboard.putNumber("Start Canrange Distance (m)", coralStartCanrange.getDistance().getValue().in(Meters));
      SmartDashboard.putNumber("Complete Canrange Distance (m)", coralCompleteCanrange.getDistance().getValue().in(Meters));
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
   * 
   * @return True if a coral is detected.
   */
  private boolean detectCoral() {
    return coralStartCanrange.getDistance().getValue().in(Meters) < coralCanrangeDistanceThreshold;
  }

  /**
   * Checks if coral is fully in the intake.
   * 
   * @return True if coral is fully in the intake.
   */
  public boolean detectCoralCompletelyIn() {
    return coralCompleteCanrange.getDistance().getValue().in(Meters) < coralCanrangeDistanceThreshold;
  }

  /**
   * Updates the intake state.
   */
  private void updateSensorStatus() {
    coralDetected = detectCoral();
    coralCompletelyIn = detectCoralCompletelyIn();
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
    inputs.coralStartCanrangeConnected = coralStartCanrange.isConnected();
    inputs.coralCompleteCanrangeConnected = coralCompleteCanrange.isConnected();
    inputs.coralStartCanrangeDistance = coralStartCanrange.getDistance().getValue().in(Meters);
    inputs.coralCompleteCanrangeDistance = coralCompleteCanrange.getDistance().getValue().in(Meters);
    inputs.coralDetected = coralDetected;
    inputs.coralDetectedCompletelyIn = coralCompletelyIn;
  }
}
