// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverJoystickConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** A command that allows the driver to control the robot using a joystick.
 * This command uses the SwerveSubsystem to drive the robot and the VisionSubsystem to aim the robot.
 * The driver can control the forward, strafe, and turn velocity of the robot using the left and right joysticks.
 * The driver can also switch between field relative and robot relative driving using a button.
 * The driver can also aim the robot using the vision system using a button.
 */
public class TeleopDriveCommand extends Command {
  private final SwerveSubsystem m_swerve;
  private final VisionSubsystem m_vision;
  private final DoubleSupplier m_forwardSupplier, m_strafeSupplier, m_turnSupplier;
  private final BooleanSupplier m_fieldRelativeSupplier, m_visionAimSupplier;


  /** 
   * Creates a new TeleopDriveCommand. Drives the robot using the {@link SwerveSubsystem} and aims the robot using the {@link VisionSubsystem}.
   *
   * @param swerve The {@link SwerveSubsystem} used by this command to drive the robot.
   * @param vision The {@link VisionSubsystem} used by this command to aim the robot.
   * @param forwardSupplier A {@link DoubleSupplier} that supplies joystick input for driving forward and backward.
   * @param strafeSupplier A {@link DoubleSupplier} that supplies joystick input for strafing the robot.
   * @param turnSupplier A {@link DoubleSupplier} that supplies joystick input for turning the robot.
   * @param fieldRelativeSupplier A {@link BooleanSupplier} that supplies whether the robot should drive in field relative
   * @param visionAimSupplier A {@link BooleanSupplier} that supplies whether the robot should aim using the vision system.
   */
  public TeleopDriveCommand(SwerveSubsystem swerve, VisionSubsystem vision, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier turnSupplier, BooleanSupplier fieldRelativeSupplier, BooleanSupplier visionAimSupplier) {
    this.m_swerve = swerve;
    this.m_vision = vision;
    this.m_forwardSupplier = forwardSupplier;
    this.m_strafeSupplier = strafeSupplier;
    this.m_turnSupplier = turnSupplier;
    this.m_fieldRelativeSupplier = fieldRelativeSupplier;
    this.m_visionAimSupplier = visionAimSupplier;

    addRequirements(this.m_swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get controller inputs
    double forward = MathUtil.applyDeadband(m_forwardSupplier.getAsDouble() * -1, DriverJoystickConstants.kLeftXDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
    double strafe = MathUtil.applyDeadband(m_strafeSupplier.getAsDouble() * -1, DriverJoystickConstants.kLeftYDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
    double turn = MathUtil.applyDeadband(m_turnSupplier.getAsDouble() * -1, DriverJoystickConstants.kRightXDeadband) * DriveConstants.kMaxAngularSpeed;

    boolean fieldRelative = m_fieldRelativeSupplier.getAsBoolean();
    boolean visionAim = m_visionAimSupplier.getAsBoolean();

    // If visionAim is true, aim using the vision system
    // if (visionAim) {
    //   var latestResultOptional = vision.getCameras().get(0).getLatestResult(); // TODO how to choose which camera to use?
    //   if (latestResultOptional.isPresent()) {
    //     var latestResult = latestResultOptional.get();
    //     if(latestResult.hasTargets()) {
    //       var target = latestResult.getBestTarget();
    //       var yaw = target.bestCameraToTarget.getRotation().getZ();
    //       turn = -MathUtil.applyDeadband(yaw, VISION_YAW_DEADBAND) * VISION_TURN_kP * swerve.getMaximumAzimuthVelocity();
    //     }
    //   }
    // }

    m_swerve.drive(forward, strafe, turn, fieldRelative);
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
