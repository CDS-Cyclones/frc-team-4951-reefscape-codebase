// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionCamera;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndGetInRangeCommand extends Command {
private final SwerveSubsystem m_Swerve;
private final VisionCamera m_VisionCamera;

  /** Creates a new AimAndGetInRangeCommand. */
  public AimAndGetInRangeCommand(SwerveSubsystem swerve, VisionCamera cam) {
    m_Swerve = swerve;
    m_VisionCamera = cam;

    addRequirements(m_Swerve, m_VisionCamera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var latestResultOptional = m_VisionCamera.getLatestResult();

    if(latestResultOptional.isPresent()) {
      var latestResult = latestResultOptional.get();

      if(latestResult.hasTargets()) {
        var target = latestResult.getBestTarget();

        var targetYaw = target.getYaw();
        var targetVisible = true;

        SmartDashboard.putNumber("id", target.getFiducialId());
        SmartDashboard.putNumber("Yaw", targetYaw);
        SmartDashboard.putNumber("Pose Ambiguity", target.getPoseAmbiguity());
      }
    }
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
