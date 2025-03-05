// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private final SparkMax pivotMotor;
  private final SparkAbsoluteEncoder pivotAbsoluteEncoder;

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
   * Checks if the pivot is out (will not be in elevator's way going up).
   *
   * @return True if the arm is out, false otherwise.
   */
  public boolean isOut() {
    return pivotAbsoluteEncoder.getPosition() >= Constants.ManipulatorConstants.kPivotMinPositionForElevatorMovement;
  }
}
