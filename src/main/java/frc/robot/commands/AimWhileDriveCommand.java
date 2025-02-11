// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.VISION_TURN_kP;
import static frc.robot.Constants.VisionConstants.VISION_YAW_DEADBAND;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverJoystickConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimWhileDriveCommand extends Command {
  private final SwerveSubsystem m_Swerve;
  private final VisionSubsystem m_Vision;
  private final CommandXboxController m_DriverController;

  double forward, strafe, turn;


  /** Creates a new AimWhileDriveCommand. */
  public AimWhileDriveCommand(SwerveSubsystem swerve, CommandXboxController controller) {
    m_Swerve = swerve;
    m_Vision = swerve.getVisionSubsystem();
    m_DriverController = controller;

    addRequirements(m_Swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forward = MathUtil.applyDeadband(m_DriverController.getLeftY() * -1, DriverJoystickConstants.kLeftXDeadband) * m_Swerve.getMaximumDriveVelocity();
    strafe = MathUtil.applyDeadband(m_DriverController.getLeftX() * -1, DriverJoystickConstants.kLeftYDeadband) * m_Swerve.getMaximumDriveVelocity();
    turn = MathUtil.applyDeadband(m_DriverController.getRightX() * -1, DriverJoystickConstants.kRightXDeadband) * DriverJoystickConstants.kTurnMultiplier * m_Swerve.getMaximumAzimuthVelocity();

    var latestResultOptional = m_Vision.getLatest2DResult();

    if(latestResultOptional.isPresent()) {
      var latestResult = latestResultOptional.get();
      if(latestResult.hasTargets()) {
        var target = latestResult.getBestTarget();
        turn =  MathUtil.applyDeadband((target.getYaw() - 0), VISION_YAW_DEADBAND) * VISION_TURN_kP * m_Swerve.getMaximumAzimuthVelocity();
      }
      m_Swerve.swerveDrive.drive(new ChassisSpeeds(forward, strafe, turn));
      System.out.println(turn);
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
