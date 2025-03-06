// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volts;

public class Pivot extends SubsystemBase {
  private final SparkMax pivotMotor;
  private final SparkAbsoluteEncoder pivotAbsoluteEncoder;
  private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
  private final SysIdRoutine pivotRoutine =
          new SysIdRoutine(
                  new SysIdRoutine.Config(),
                  new SysIdRoutine.Mechanism(
                          this::setVoltage,
                          this::logMotors,
                          this
                  )
          );

  /** Creates a new Pivot subsystem. */
  public Pivot() {
    pivotMotor = new SparkMax(Constants.ManipulatorConstants.kPivotMotorId, MotorType.kBrushless);

    pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();

    pivotMotor.configure(Constants.ManipulatorConstants.kPivotMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // print to Shuffleboard the encoder value
    try {
      SmartDashboard.putNumber("pivot/absolute-encoder", pivotAbsoluteEncoder.getPosition());
    } catch (Exception e) {}
  }

  /**
   * Sets the speed of the pivot motor.
   *
   * @param speed The speed to set the motor to. Positive values move the pivot out, negative values move the arm in.
   */
  public void setSpeed(double speed) {
    pivotMotor.set(speed);
  }

  /**
   * Sets the voltage of the pivot motor.
   *
   * @param voltage The {@link Voltage} to set the motor to in volts. Positive values move the pivot out, negative values move the arm in.
   */
  public void setVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  /**
   * Sets the voltage of the pivot motor.
   *
   * @param voltage The {@link Voltage} to set the motor to in volts. Positive values move the pivot out, negative values move the arm in.
   */
  public void setVoltage(Voltage voltage) {
    pivotMotor.setVoltage(voltage);
  }

  /**
   * Stops the pivot motor.
   */
  public void stop() {
    pivotMotor.stopMotor();
  }

  /**
   * Gets the absolute position of the pivot.
   *
   * @return The absolute position of the pivot.
   */
  public double getPosition() {
    return pivotAbsoluteEncoder.getPosition();
  }

  /**
     * Calculate the feedforward voltage for the elevator.
          *
          * @param velocity The velocity of the elevator.
          *
          * @return The feedforward voltage as a double.
          */
  public double calculateFeedforward(double velocity, double acceleration) {
    return feedforward.calculate(velocity, acceleration);
  }

  /**
   * Checks if the pivot is out (will not be in elevator's way going up).
   *
   * @return True if the arm is out, false otherwise.
   */
  public boolean isOut() {
    return pivotAbsoluteEncoder.getPosition() >= Constants.ManipulatorConstants.kPivotMinPositionForElevatorMovement;
  }

  public void logMotors(SysIdRoutineLog log) {
    // Calculate the raw voltage
    double motorVoltage1 = pivotMotor.get() * RobotController.getBatteryVoltage();

    // Convert it to a Voltage type
    Voltage voltage1 = Voltage.ofBaseUnits(motorVoltage1, Volts);

    // Log the voltage
    log.motor("pivot-motor").voltage(voltage1);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotRoutine.dynamic(direction);
  }
}
