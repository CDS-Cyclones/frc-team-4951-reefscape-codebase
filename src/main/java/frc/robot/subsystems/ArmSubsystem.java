// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armMotor;
  private final SparkBaseConfig armConfig;
  private final SparkAbsoluteEncoder armAbsoluteEncoder;


  /** Creates a new ElevatorSubsystem. */
  public ArmSubsystem() {
    armMotor = new SparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);

    armConfig = new SparkMaxConfig();
    armConfig.idleMode(ArmConstants.kIdleMode);
    armConfig.inverted(ArmConstants.kInverted);

    armMotor.configure(armConfig, null, null);

    armAbsoluteEncoder = armMotor.getAbsoluteEncoder();
  }


  @Override
  public void periodic() {
    // print to Shuffleboard the encoder value
    try {
    SmartDashboard.putNumber("Arm Encoder", armAbsoluteEncoder.getPosition());

    } catch (Exception e) {
      System.out.println("Error in ArmSubsystem.java");
    }
  }


  /** 
   * Sets the speed of the arm motor.
   * 
   * @param speed The speed to set the motor to. Positive values move the arm out, negative values move the arm in.
   */
  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }


  /** Stops the arm motor. */
  public void stopArm() {
    armMotor.stopMotor();
  }


  /** 
   * Checks if the arm is out.
   * 
   * @return True if the arm is out, false if the arm is less than fully out.
   */
  public boolean isArmOut() {
    return armAbsoluteEncoder.getPosition() >= ArmConstants.kArmMax;
  }


  /** 
   * Gets the absolute position of the arm.
   * 
   * @return The absolute position of the arm.
   */
  public double getArmPosition() {
    return armAbsoluteEncoder.getPosition();
  }
}
