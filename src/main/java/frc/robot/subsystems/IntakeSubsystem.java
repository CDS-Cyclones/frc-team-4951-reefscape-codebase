// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;
  private final SparkBaseConfig intakeConfig;


  /** Creates a new ElevatorSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new SparkMax(56, MotorType.kBrushless);

    intakeEncoder = intakeMotor.getEncoder();

    intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);

    intakeMotor.configure(intakeConfig, null, null);
  }


  @Override
  public void periodic() {
    // print to Shuffleboard the encoder value
    try {
    SmartDashboard.putNumber("Intake Encoder", intakeEncoder.getPosition());

    } catch (Exception e) {
      System.out.println("Error in IntakeSubsystem.java");
    }

  }


  /** 
   * Sets the speed of the intake motor.
   * 
   * @param speed The speed to set the motors to. Positive values move the elevator up, negative values move the elevator down.
   */
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }


  /** Stops the intake motors. */
  public void stopIntake() {
    intakeMotor.stopMotor();
  }


  /** 
   * Gets the position of the intake.
   * 
   * @return The position of the intake.
   */
  public double getIntakePosition() {
    return intakeEncoder.getPosition();
  }
}
