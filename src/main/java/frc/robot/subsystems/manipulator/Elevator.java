// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final SparkMax elevatorMotor1, elevatorMotor2;
  private final RelativeEncoder elevatorEncoder1, elevatorEncoder2;

  /** Creates a new Elevator subsystem. */
  public Elevator() {
    elevatorMotor1 = new SparkMax(Constants.ManipulatorConstants.kElevatorMotor1Id, MotorType.kBrushless);
    elevatorMotor2 = new SparkMax(Constants.ManipulatorConstants.kElevatorMotor2Id, MotorType.kBrushless);

    elevatorEncoder1 = elevatorMotor1.getEncoder();
    elevatorEncoder2 = elevatorMotor2.getEncoder();

    elevatorMotor1.configure(Constants.ManipulatorConstants.kElevatorMotor1Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    elevatorMotor2.configure(Constants.ManipulatorConstants.kElevatorMotor2Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
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
}
