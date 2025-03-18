// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.OIUtil;

/**
 * Field relative drive command using two joysticks (controlling linear and angular velocities).
 */
public class JoystickDriveCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final BooleanSupplier fieldRelative;

  /** Creates a new JoystickDriveCommand. */
  public JoystickDriveCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier fieldRelative) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.fieldRelative = fieldRelative;

    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get linear velocity
    Translation2d linearVelocity = OIUtil.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Apply rotation deadband
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), OIConstants.kJoystickAxisDeadband);

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds = new ChassisSpeeds(
      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
      omega * drive.getMaxAngularSpeedRadPerSec()
    );
    boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

    // Determine robot angle based on whether the robot is in field relative mode or not
    // Always keeping angle as 0 is analogous to setting the robot as robot relative
    Rotation2d robotAngle;
    if (fieldRelative.getAsBoolean())
      robotAngle = isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation();
    else
      robotAngle = new Rotation2d();

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
