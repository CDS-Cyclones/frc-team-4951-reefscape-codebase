// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.MutableFieldPose.FieldPose;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.OIUtil;
import static frc.robot.Constants.DriveConstants.*;

/*
 * This command allows manual control of the robot's x and y directions while using a ProfiledPIDController to adjust its orientation (omega) to a desired field pose.
 * Additionally, if the vision system detects the desired tag and the robot is within a certain range, the command will override manual control and autonomously navigate the robot to the target.
 */
public class VisionAssistedDriveToPoseCommand extends Command {
  private final Drive drive;
  private final Vision vision;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Supplier<FieldPose> desiredFieldPoseSupplier;

  private final ProfiledPIDController omegaController;

  /** Creates a new JoystickDriveAtAngleCommand. */
  public VisionAssistedDriveToPoseCommand(Drive drive, Vision vision, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<FieldPose> desiredFieldPoseSupplier) {
    this.drive = drive;
    this.vision = vision;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.desiredFieldPoseSupplier = desiredFieldPoseSupplier;

    addRequirements(this.drive, this.vision);

    omegaController = new ProfiledPIDController(omegaPPIDCKp, 0.0, omegaPPIDCKd, new TrapezoidProfile.Constraints(omegaPPIDCMaxVel, omegaPPIDCMaxAccel));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    omegaController.reset(drive.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get linear velocity from joystick (vision might override later)
    Translation2d linearVelocity = OIUtil.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Calculate angular speed
    double omega = omegaController.calculate(drive.getRotation().getRadians(), desiredFieldPoseSupplier.get().getDesiredRotation2d().getRadians());

    // If vision system detects target and robot is within a certain range, override manual control
    // if (LimelightHelpers.getTV(cameraName) && LimelightHelpers.getFiducialID(cameraName) == desiredFieldPoseSupplier.get().getTagId()) {
    //   // Calculate linear velocity from vision
    //   linearVelocity = vision.getLinearVelocityToTarget();
    //   // Calculate angular velocity from vision
    //   omega = vision.getAngularVelocityToTarget();
    // }

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =  new ChassisSpeeds(
      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
      omega
    );
    boolean isFlipped =  DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
              
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds,
      isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()
    ));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
