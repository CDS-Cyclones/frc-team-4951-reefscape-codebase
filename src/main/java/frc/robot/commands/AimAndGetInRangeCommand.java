// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverJoystickConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionCamera;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndGetInRangeCommand extends Command {
  private final SwerveSubsystem m_Swerve;
  private final CommandXboxController m_DriverController;

  double forward, strafe, turn, targetRange;

  boolean isRunning = true;


  /** Creates a new AimAndGetInRangeCommand. */
  public AimAndGetInRangeCommand(SwerveSubsystem swerve, CommandXboxController controller) {
    m_Swerve = swerve;
    m_DriverController = controller;

    addRequirements(m_Swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    forward = 0; // MathUtil.applyDeadband(m_DriverController.getLeftY() * -1, DriverJoystickConstants.kLeftXDeadband) * m_Swerve.swerveDrive.getMaximumChassisVelocity();
    strafe = MathUtil.applyDeadband(m_DriverController.getLeftX() * -1, DriverJoystickConstants.kLeftYDeadband) * m_Swerve.swerveDrive.getMaximumChassisVelocity();
    turn = MathUtil.applyDeadband(m_DriverController.getRightX() * -1, DriverJoystickConstants.kRightXDeadband) * DriverJoystickConstants.kTurnMultiplier * m_Swerve.swerveDrive.getMaximumChassisAngularVelocity();

    var latestResultOptional = m_Swerve.photonCamera.getLatestResult();
    if(latestResultOptional.isPresent()) {
      var latestResult = latestResultOptional.get();
      if(latestResult.hasTargets()) {
        var target = latestResult.getBestTarget();
        // turn = -1.0 * MathUtil.applyDeadband(target.getYaw(), Math.PI) * m_Swerve.swerveDrive.getMaximumChassisAngularVelocity();
        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
            0.23, // Measured with a tape measure, or in CAD.
            0.24, // From 2024 game manual for ID 7
            Units.degreesToRadians(0), // Measured with a protractor, or in CAD.
            Units.degreesToRadians(target.getPitch()));
        }
        System.out.println(targetRange);
        if(targetRange > 0.5) {
            strafe = targetRange * 0.5 * m_Swerve.swerveDrive.getMaximumChassisVelocity();
        } else {
            strafe = 0;
        }
      m_Swerve.swerveDrive.drive(new ChassisSpeeds(forward, strafe, turn * 0.3));
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
