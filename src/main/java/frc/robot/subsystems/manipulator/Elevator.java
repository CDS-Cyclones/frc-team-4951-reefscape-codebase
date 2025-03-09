// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase {
  private final SparkMax elevatorMotor1, elevatorMotor2;
  private final RelativeEncoder elevatorEncoder1, elevatorEncoder2;
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0, 0);
  private final SysIdRoutine elevatorRoutine =
          new SysIdRoutine(
                  new SysIdRoutine.Config(),
                  new SysIdRoutine.Mechanism(
                          this::setVoltage,
                          this::logMotors,
                          this
                  )
          );

  /** Creates a new Elevator subsystem. */
  public Elevator() {
    elevatorMotor1 = new SparkMax(Constants.ManipulatorConstants.kElevatorMotor1Id, MotorType.kBrushless);
    elevatorMotor2 = new SparkMax(Constants.ManipulatorConstants.kElevatorMotor2Id, MotorType.kBrushless);

    elevatorEncoder1 = elevatorMotor1.getEncoder();
    elevatorEncoder2 = elevatorMotor2.getEncoder();

    elevatorMotor1.configure(Constants.ManipulatorConstants.kElevatorMotor1Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    elevatorMotor2.configure(Constants.ManipulatorConstants.kElevatorMotor2Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    zeroEncoders();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // print to Shuffleboard the encoder values
    try {
    SmartDashboard.putNumber("elevator/encoder-1", elevatorEncoder1.getPosition());
    SmartDashboard.putNumber("elevator/encoder-2", elevatorEncoder2.getPosition());
    } catch (Exception e) {}
  }

  /**
   * Sets the speed of the motors.
   *
   * @param speed The speed to set the motors to. Positive values move the elevator up, negative values move the elevator down.
   */
  public void setSpeed(double speed) {
    elevatorMotor1.set(speed);
    elevatorMotor2.set(speed);
  }

  /**
   * Sets the voltage of the motors.
   *
   * @param voltage The voltage to set the motors to in volts. Positive values move the elevator up, negative values move the elevator down.
   */
  public void setVoltage(double voltage) {
    elevatorMotor1.setVoltage(voltage);
    elevatorMotor2.setVoltage(voltage);
  }

  /**
   * Sets the voltage of the motors.
   *
   * @param voltage The {@link Voltage} to set the motors to in volts. Positive values move the elevator up, negative values move the elevator down.
   */
  public void setVoltage(Voltage voltage) {
    elevatorMotor1.setVoltage(voltage);
    elevatorMotor2.setVoltage(voltage);
  }

  /** Stops the elevator motors. */
  public void stop() {
    elevatorMotor1.stopMotor();
    elevatorMotor2.stopMotor();
  }

  /**
   * Gets the position of the elevator.
   *
   * @return The position of the elevator.
   */
  public double getPosition() {
    return (elevatorEncoder1.getPosition() + elevatorEncoder2.getPosition()) / 2;
  }

  /**
   * Reset the encoder positions of the elevator to zero.
   */
  public void zeroEncoders() {
    elevatorEncoder1.setPosition(0);
    elevatorEncoder2.setPosition(0);
  }

  /**
   * Calculate the feedforward voltage for the elevator.
   *
   * @param velocity The velocity of the elevator.
   *
   * @return The feedforward voltage as a double.
   */
  public double calculateFeedforward(double velocity) {
    return feedforward.calculate(velocity);
  }

  public void logMotors(SysIdRoutineLog log) {
    // Calculate the raw voltage
    double motorVoltage1 = elevatorMotor1.get() * RobotController.getBatteryVoltage();
    double motorVoltage2 = elevatorMotor2.get() * RobotController.getBatteryVoltage();

    // Convert it to a Voltage type
    Voltage voltage1 = Voltage.ofBaseUnits(motorVoltage1, Volts);
    Voltage voltage2 = Voltage.ofBaseUnits(motorVoltage2, Volts);

    // Log the voltage
    log.motor("shooter-motor-1").voltage(voltage1);
    log.motor("shooter-motor-2").voltage(voltage2);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return elevatorRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return elevatorRoutine.dynamic(direction);
  }
}
