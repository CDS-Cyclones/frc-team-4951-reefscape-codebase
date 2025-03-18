// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotStateConstants.FieldPose;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import static frc.robot.Constants.DriveConstants.*;

/*
 * */
public class AutoDriveToPoseCommand extends Command {
  private final Drive drive;
  private final Vision vision;
  private final FieldPose desiredFieldPose;

  private ProfiledPIDController angleController;
  private PIDController translationXController;
  private PIDController translationYController;

  private ChassisSpeeds speeds;
  private boolean isFlipped;

  /** Creates a new JoystickDriveAtAngleCommand. */
  public AutoDriveToPoseCommand(Drive drive, Vision vision, FieldPose desiredFieldPose) {
    this.drive = drive;
    this.vision = vision;
    this.desiredFieldPose = desiredFieldPose;

    addRequirements(this.drive, this.vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(desiredFieldPose.isOrientationOnly())
      throw new IllegalArgumentException("Desired field pose must have a translation component for AutoDriveToPoseCommand");

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

    angleController.setTolerance(anglePIDTolerance.getAsDouble());
    translationXController.setTolerance(translationPIDTolerance.getAsDouble());
    translationYController.setTolerance(translationPIDTolerance.getAsDouble());

    // Check if red alliance
    isFlipped =  DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d pose = desiredFieldPose.getDesiredPose();
      
    double velocityX = translationXController.calculate(drive.getPose().getTranslation().getX(), pose.getTranslation().getX());
    double velocityY = translationYController.calculate(drive.getPose().getTranslation().getY(), pose.getTranslation().getY());
    double omega = angleController.calculate(drive.getRotation().getRadians(), desiredFieldPose.getDesiredRotation2d().getRadians());
  
    speeds = new ChassisSpeeds(isFlipped ? -velocityX : velocityX, isFlipped ? -velocityY : velocityY, omega);

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command shyould end.
  @Override
  public boolean isFinished() {
    return translationXController.atSetpoint() && translationYController.atSetpoint() && angleController.atSetpoint();
  }
}
