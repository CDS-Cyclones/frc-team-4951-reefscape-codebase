// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax elevatorMotor1, elevatorMotor2;
  private final RelativeEncoder elevatorEncoder1, elevatorEncoder2;
  private final SparkBaseConfig elevatorConfig;


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor1 = new SparkMax(ElevatorConstants.kElevatorMotor1, MotorType.kBrushless);
    elevatorMotor2 = new SparkMax(ElevatorConstants.kElevatorMotor2, MotorType.kBrushless);

    elevatorEncoder1 = elevatorMotor1.getEncoder();
    elevatorEncoder2 = elevatorMotor2.getEncoder();

    elevatorConfig = new SparkMaxConfig();
    elevatorConfig.idleMode(ElevatorConstants.kIdleMode);

    elevatorMotor1.configure(elevatorConfig, null, null);
    elevatorMotor2.configure(elevatorConfig, null, null);
  }


  @Override
  public void periodic() {
    // print to Shuffleboard the encoder vaklues
    try {
    SmartDashboard.putNumber("Elevator Encoder 1", elevatorEncoder1.getPosition());
    SmartDashboard.putNumber("Elevator Encoder 2", elevatorEncoder2.getPosition());
    } catch (Exception e) {
      System.out.println("Error in ElevatorSubsystem.java");
    }

  }


  /**
   * Sets the speed of the elevator motors.
   *
   * @param speed The speed to set the motors to. Positive values move the elevator up, negative values move the elevator down.
   */
  public void setElevatorSpeed(double speed) {
    elevatorMotor1.set(speed);
    elevatorMotor2.set(speed);
  }


  /** Stops the elevator motors. */
  public void stopElevator() {
    elevatorMotor1.stopMotor();
    elevatorMotor2.stopMotor();
  }


  /**
   * Gets the position of the elevator.
   *
   * @return The position of the elevator.
   */
  public double getElevatorPosition() {
    return (elevatorEncoder1.getPosition() + elevatorEncoder2.getPosition()) / 2;
  }
}
