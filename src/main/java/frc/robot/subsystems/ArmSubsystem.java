// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armMotor;
  private final RelativeEncoder armEncoder;
  private final SparkBaseConfig armConfig;
  private final SparkAbsoluteEncoder armAbsoluteEncoder;


  /** Creates a new ElevatorSubsystem. */
  public ArmSubsystem() {
    armMotor = new SparkMax(56, MotorType.kBrushless);

    armEncoder = armMotor.getEncoder();

    armConfig = new SparkMaxConfig();
    armConfig.idleMode(IdleMode.kBrake);

    armMotor.configure(armConfig, null, null);

    armAbsoluteEncoder = armMotor.getAbsoluteEncoder();
  }


  @Override
  public void periodic() {
    // print to Shuffleboard the encoder value
    try {
    SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition());

    } catch (Exception e) {
      System.out.println("Error in IntakeSubsystem.java");
    }

  }


  /** 
   * Sets the speed of the arm motor.
   * 
   * @param speed The speed to set the motors to. Positive values move the arm up, negative values move the arm down.
   */
  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }


  /** Stops the intake motors. */
  public void stopArm() {
    armMotor.stopMotor();
  }


  /** 
   * Gets the position of the arm.
   * 
   * @return The position of the arm.
   */
  public double getArmPosition() {
    return armAbsoluteEncoder.getPosition();
  }
}
