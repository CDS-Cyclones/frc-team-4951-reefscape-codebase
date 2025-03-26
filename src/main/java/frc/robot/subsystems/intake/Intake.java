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
  private final CANrange coralInflowCanrange = new CANrange(coralInflowCanrangeCanId, coralInflowCanrangeCanBus);
  private final CANrange coralOutflowCanrange = new CANrange(coralOutflowCanrangeCanId, coralOutflowCanrangeCanBus);

  @Getter boolean coralDetectedAtInflow = false;
  @Getter boolean coralDetectedAtOutflow = false;
  @Getter boolean intakeContainsCoral = false;

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
      SmartDashboard.putString("Mutables/Intake Action", RobotStateManager.getDesiredIntakeAction().toString());
      SmartDashboard.putBoolean("Coral Detected At Inflow", coralDetectedAtInflow);
      SmartDashboard.putBoolean("Coral Detected At Outflow", coralDetectedAtOutflow);
      SmartDashboard.putBoolean("Intake Contains Coral", intakeContainsCoral);
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
   *  Checks if Canrange sensor detects coral in the intake on the inflow side.
   * 
   * @return True if coral is detected in the intake on the inflow side.
   */
  private boolean coralDetectedAtInflow() {
    return coralInflowCanrange.getDistance().getValue().in(Meters) < coralCanrangeDistanceThreshold;
  }

  /**
   * Checks if Canrange sensor detects coral in the intake on the outflow side.
   * 
   * @return True if coral is detected in the intake on the outflow side.
   */
  public boolean coralDetectedAtOutflow() {
    return coralOutflowCanrange.getDistance().getValue().in(Meters) < coralCanrangeDistanceThreshold;
  }

  /**
   * Updates the intake state.
   */
  private void updateSensorStatus() {
    coralDetectedAtInflow = coralDetectedAtInflow();
    coralDetectedAtOutflow = coralDetectedAtOutflow();

    // If coral is detected at either the inflow or outflow, the intake contains coral.
    intakeContainsCoral = coralDetectedAtInflow || coralDetectedAtOutflow;
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
    inputs.coralInflowCanrangeConnected = coralInflowCanrange.isConnected();
    inputs.coralOutflowCanrangeConnected = coralOutflowCanrange.isConnected();
    inputs.coralInflowCanrangeDistance = coralInflowCanrange.getDistance().getValue().in(Meters);
    inputs.coralOutflowCanrangeDistance = coralOutflowCanrange.getDistance().getValue().in(Meters);
    inputs.coralDetectedOnInflow = coralDetectedAtInflow();
    inputs.coralDetectedOnOutflow = coralDetectedAtOutflow();
    inputs.intakeContainsCoral = intakeContainsCoral;
  }
}
