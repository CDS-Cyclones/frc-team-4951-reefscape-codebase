// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

public class SwerveSubsystem extends SubsystemBase {
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;

  // PID NetworkTableEntries
  private NetworkTableEntry driveP, driveI, driveD, turnP, turnI, turnD;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Initialize NetworkTableEntries
    driveP = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Drive PID P");
    driveI = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Drive PID I");
    driveD = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Drive PID D");
    turnP = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Turn PID P");
    turnI = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Turn PID I");
    turnD = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Turn PID D");
  
    // Set default PID values
    driveP.setDouble(swerveDrive.getModules()[0].getDrivePIDF().p);
    driveI.setDouble(swerveDrive.getModules()[0].getDrivePIDF().i);
    driveD.setDouble(swerveDrive.getModules()[0].getDrivePIDF().d);
    turnP.setDouble(swerveDrive.getModules()[0].getAnglePIDF().p);
    turnI.setDouble(swerveDrive.getModules()[0].getAnglePIDF().i);
    turnD.setDouble(swerveDrive.getModules()[0].getAnglePIDF().d);
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(
        new Translation2d(
          translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumVelocity()
        ),
        angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
        true,
        false);
    });
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Gyro data
    SmartDashboard.putNumber("Gyro Angle (degrees)", swerveDrive.getYaw().getDegrees());
    
    // Turn encoders
    SmartDashboard.putNumber("Front Left Turn Encoder (degrees)", swerveDrive.getModules()[0].getRelativePosition());
    SmartDashboard.putNumber("Front Right Turn Encoder (degrees)", swerveDrive.getModules()[1].getRelativePosition());
    SmartDashboard.putNumber("Back Left Turn Encoder (degrees)", swerveDrive.getModules()[2].getRelativePosition());
    SmartDashboard.putNumber("Back Right Turn Encoder (degrees)", swerveDrive.getModules()[3].getRelativePosition());
    
    // Absolute encoders
    SmartDashboard.putNumber("Front Left Absolute Encoder (degrees) [0, 360]", swerveDrive.getModules()[0].getAbsolutePosition());
    SmartDashboard.putNumber("Front Right Absolute Encoder (degrees) [0, 360]", swerveDrive.getModules()[1].getAbsolutePosition());
    SmartDashboard.putNumber("Rear Left Absolute Encoder (degrees) [0, 360]", swerveDrive.getModules()[2].getAbsolutePosition());
    SmartDashboard.putNumber("Rear Right Absolute Encoder (degrees) [0, 360]", swerveDrive.getModules()[3].getAbsolutePosition());
    
    // PID values
    double newDriveP = driveP.getDouble(swerveDrive.getModules()[0].getDrivePIDF().p);
    double newDriveI = driveI.getDouble(swerveDrive.getModules()[0].getDrivePIDF().i);
    double newDriveD = driveD.getDouble(swerveDrive.getModules()[0].getDrivePIDF().d);
    double newTurnP = turnP.getDouble(swerveDrive.getModules()[0].getAnglePIDF().p);
    double newTurnI = turnI.getDouble(swerveDrive.getModules()[0].getAnglePIDF().i);
    double newTurnD = turnD.getDouble(swerveDrive.getModules()[0].getAnglePIDF().d);

    PIDFConfig drivePID = new PIDFConfig(newDriveP, newDriveI, newDriveD);
    PIDFConfig turnPID = new PIDFConfig(newTurnP, newTurnI, newTurnD);
    
    // Update PID controller values if changed
    for (int i=0; i<4; i++)
    {
    swerveDrive.getModules()[i].setAnglePIDF(turnPID);
    swerveDrive.getModules()[i].setDrivePIDF(drivePID);
    }

    // Log the PID values on the SmartDashboard
    SmartDashboard.putNumber("Drive PID P", newDriveP);
    SmartDashboard.putNumber("Drive PID I", newDriveI);
    SmartDashboard.putNumber("Drive PID D", newDriveD);
    SmartDashboard.putNumber("Turn PID P", newTurnP);
    SmartDashboard.putNumber("Turn PID I", newTurnI);
    SmartDashboard.putNumber("Turn PID D", newTurnD);
  }
}
