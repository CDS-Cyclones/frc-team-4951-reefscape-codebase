// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkBaseConfig intakeConfig;


  /** Creates a new ElevatorSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);

    intakeConfig = new SparkMaxConfig();
    intakeConfig.inverted(IntakeConstants.kInverted);
    intakeConfig.idleMode(IntakeConstants.kIdleMode);

    intakeMotor.configure(intakeConfig, null, null);
  }


  // This method will be called once per scheduler run
  @Override
  public void periodic() {
  }


  /** 
   * Sets the speed of the intake motor.
   * 
   * @param speed The speed to set the motors to. Positive values outtake, negative values intake.
   */
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }


  /** Stops the intake motor. */
  public void stopIntake() {
    intakeMotor.stopMotor();
  }
}
