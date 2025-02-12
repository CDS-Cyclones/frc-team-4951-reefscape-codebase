// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.photonvision.PhotonUtils;

import static frc.robot.Constants.VisionConstants.*;

public class AimAndGetInRangeCommand extends Command {
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;

  double turn = 0, range = 0;


  /** Creates a new AimAndGetInRangeCommand.
   *
   * @param swerve The SwerveSubsystem used by this command to drive the robot.
   */
  public AimAndGetInRangeCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.vision = swerve.getVisionSubsystem();

    addRequirements(this.swerve, this.vision);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double range, turn = 0, forward = 0;
    var latestResultOptional = vision.getLatest2DResult();
    if (latestResultOptional.isPresent()) {
      var latestResult = latestResultOptional.get();
      var target = latestResult.getBestTarget();

      // Calculate the turn speed
      turn = -MathUtil.applyDeadband(target.getYaw(), VISION_YAW_DEADBAND) * VISION_TURN_kP * swerve.getMaximumAzimuthVelocity();

      // Calculate the forward speed
      range = PhotonUtils.calculateDistanceToTargetMeters(
        Constants.VisionConstants.VisionCameraInfo.PRIMARY.botToCam.getTranslation().getZ(),
        Constants.VisionConstants.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
        Constants.VisionConstants.VisionCameraInfo.PRIMARY.botToCam.getRotation().getZ(),
        Units.degreesToRadians(target.getPitch())
      );
      forward = (range - DESIRED_RANGE) * VISION_FORWARD_kP * swerve.getMaximumDriveVelocity();
    }

    // Drive the robot
    swerve.drive(forward, 0, turn, false, true);
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
