// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWheels extends SubsystemBase {
  private final SparkMax intakeMotor;

  /** Creates a new IntakeWheels subsystem. */
  public IntakeWheels() {
    intakeMotor = new SparkMax(Constants.ManipulatorConstants.kIntakeWheelsMotorId, MotorType.kBrushless);

    intakeMotor.configure(Constants.ManipulatorConstants.kIntakeWheelsMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {}

  /**
   * Sets the speed of the intake motor.
   *
   * @param speed The speed to set the motor to.
   */
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Stops the intake motor.
   */
  public void stop() {
    intakeMotor.stopMotor();
  }
}
