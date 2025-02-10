// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverJoystickConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionCamera;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimWhileDriveCommand extends Command {
  private final SwerveSubsystem m_Swerve;
  private final VisionCamera m_VisionCamera;
  private final CommandXboxController m_DriverController;

  double forward, strafe, turn;

  boolean isRunning = true;


  /** Creates a new AimWhileDriveCommand. */
  public AimWhileDriveCommand(SwerveSubsystem swerve, VisionCamera cam, CommandXboxController controller) {
    m_Swerve = swerve;
    m_VisionCamera = cam;
    m_DriverController = controller;

    addRequirements(m_Swerve, m_VisionCamera);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    forward = MathUtil.applyDeadband(m_DriverController.getLeftY() * -1, DriverJoystickConstants.kLeftXDeadband) * m_Swerve.swerveDrive.getMaximumChassisVelocity();
    strafe = MathUtil.applyDeadband(m_DriverController.getLeftX() * -1, DriverJoystickConstants.kLeftYDeadband) * m_Swerve.swerveDrive.getMaximumChassisVelocity();
    turn = MathUtil.applyDeadband(m_DriverController.getRightX() * -1, DriverJoystickConstants.kRightXDeadband) * DriverJoystickConstants.kTurnMultiplier * m_Swerve.swerveDrive.getMaximumChassisAngularVelocity();
  
    var latestResultOptional = m_VisionCamera.getLatestResult();
    if(latestResultOptional.isPresent()) {
      var latestResult = latestResultOptional.get();
      if(latestResult.hasTargets()) {
        var target = latestResult.getBestTarget();
        turn = -1.0 * MathUtil.applyDeadband(target.getYaw(), Math.PI) * m_Swerve.swerveDrive.getMaximumChassisAngularVelocity();
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
