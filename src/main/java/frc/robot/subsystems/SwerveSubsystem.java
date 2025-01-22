// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  private final File                      swerveJsonDirectory;
  private final SwerveDrive               swerveDrive;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    // Enable high telemetry (allows to see more data in ShuffleBoard)
    SwerveDriveTelemetry.verbosity=TelemetryVerbosity.HIGH;

    // Load config from JSOn files
    swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");

    // Initialize swerve drive
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Pushes the absolute encoder offsets to motor controllers
    swerveDrive.pushOffsetsToEncoders();

    // Set motors to brake mode
    setMotorBrake(true);
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction (forward).
   * @param translationY     Translation in the Y direction (left).
   * @param angularRotationX Rotation of the robot to set
   * @param fieldRelative    If true, robot will drive in field-oriented mode, else in robot-oriented mode.
   * @param inAuton          If in autonomous will use close-loop velocity(higher precision), in teleop will use open-loop velocity(higher responsiveness).
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX, boolean fieldRelative, boolean inAuton)
  {
    return run(() -> {
      // Make the robot drive
      swerveDrive.drive(
        new Translation2d(
          translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
        ),
        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
        fieldRelative,
        !inAuton);
    });
  }

//   /**
//    * Command to drive the robot using translative values and heading as angular velocity.
//    *
//    * @param translationX     Translation in the X direction.
//    * @param translationY     Translation in the Y direction.
//    * @param angularRotationX Rotation of the robot to set
//    * @return Drive command.
//    */
//   public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
//   {
//     return run(() -> {
//       // Make the robot move
//       swerveDrive.drive(
//         new Translation2d(
//           translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
//           translationY.getAsDouble() * swerveDrive.getMaximumVelocity()
//         ),
//         angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
//         true,
//         false);
//     });
//   }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}