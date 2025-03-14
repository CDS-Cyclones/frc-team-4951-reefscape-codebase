// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.mutables.MutableFieldPose.FieldPose;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.OIUtil;
import frc.robot.mutables.MutableCandleState;
import frc.robot.mutables.MutableCandleState.CandleState;
import static frc.robot.Constants.DriveConstants.*;

/*
 * This command allows manual control of the robot's x and y directions while using a {@link ProfiledPIDController} to adjust its orientation (omega) to a desired field pose.
 * Additionally, if the vision system detects the desired tag and the robot is within a certain range, the command will override manual control and autonomously navigate the robot to the target.
 */
public class VisionAssistedDriveToPoseCommand extends Command {
  private final Drive drive;
  private final Vision vision;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Supplier<FieldPose> desiredFieldPoseSupplier;

  private ProfiledPIDController angleController;
  private PIDController translationXController;
  private PIDController translationYController;

  private ChassisSpeeds speeds;
  private boolean isFlipped;
  private boolean hasDetectedDesiredTag;

  /** Creates a new JoystickDriveAtAngleCommand. */
  public VisionAssistedDriveToPoseCommand(Drive drive, Vision vision, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<FieldPose> desiredFieldPoseSupplier) {
    this.drive = drive;
    this.vision = vision;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.desiredFieldPoseSupplier = desiredFieldPoseSupplier;

    addRequirements(this.drive, this.vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController = new ProfiledPIDController(
      anglePIDCKp.getAsDouble(),
      0.0,
      anglePIDCKd.getAsDouble(),
      new TrapezoidProfile.Constraints(
        anglePIDCMaxVel.getAsDouble(),
        anglePIDCMaxAccel.getAsDouble()
      )
    );
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    translationXController = new PIDController(
      translationPIDCKp.getAsDouble(),
      0.0,
      translationPIDCKd.getAsDouble()
    );
    translationYController = new PIDController(
      translationPIDCKp.getAsDouble(),
      0.0,
      translationPIDCKd.getAsDouble()
    );

    angleController.reset(drive.getRotation().getRadians());
    translationXController.reset();
    translationYController.reset();

    // Check if red alliance
    isFlipped =  DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

    hasDetectedDesiredTag = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate angular speed
    double omega = angleController.calculate(drive.getRotation().getRadians(), desiredFieldPoseSupplier.get().getDesiredRotation2d().getRadians());

    if(!hasDetectedDesiredTag && !desiredFieldPoseSupplier.get().isOrientationOnly()) {
      // Check if vision system detects desired tag
      int[] detectedTagIds = vision.getTagIds(0);
      for (int tagId : detectedTagIds) {
        if (tagId == desiredFieldPoseSupplier.get().getTagId()) {
          hasDetectedDesiredTag = true;
          MutableCandleState.setMutableCandleState(CandleState.TARGET_FOUND);
          break;
        }
      }
    }

    // If vision system detects desired tag, poseestimation is accurate enough to autonomous navigate to target
    if(hasDetectedDesiredTag && !desiredFieldPoseSupplier.get().isOrientationOnly()) {
      if(angleController.atSetpoint() && translationXController.atSetpoint() && translationYController.atSetpoint()) {
        MutableCandleState.setMutableCandleState(CandleState.AT_POSE);
      }

      Pose3d pose = desiredFieldPoseSupplier.get().getDesiredPose();
      
      // Calculate translation speed
      double velocityX = translationXController.calculate(drive.getPose().getTranslation().getX(), pose.getTranslation().getX());
      double velocityY = translationYController.calculate(drive.getPose().getTranslation().getY(), pose.getTranslation().getY());
  
      speeds =  new ChassisSpeeds(
        isFlipped ? -velocityX : velocityX,
        isFlipped ? -velocityY : velocityY,
        omega
      );

      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    
      return;
    } else {
      // Get linear velocity
      Translation2d linearVelocity = OIUtil.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

      speeds = new ChassisSpeeds(
        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
        omega
      );

      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    MutableCandleState.setMutableCandleState(CandleState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
