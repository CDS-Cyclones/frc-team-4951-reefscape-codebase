// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.VISION_TURN_kP;
import static frc.robot.Constants.VisionConstants.VISION_YAW_DEADBAND;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverJoystickConstants;
import frc.robot.subsystems.SwerveSubsystem;
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
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final DoubleSupplier forwardSupplier, strafeSupplier, turnSupplier;
  private final BooleanSupplier fieldRelativeSupplier, visionAimSupplier;


  /** Creates a new TeleopDriveCommand.
   *
   * @param swerve The {@link SwerveSubsystem} used by this command to drive the robot.
   * @param vision The {@link VisionSubsystem} used by this command to aim the robot.
   * @param forwardSupplier A DoubleSupplier that supplies the forward velocity of the robot.
   * @param strafeSupplier A DoubleSupplier that supplies the strafe velocity of the robot.
   * @param turnSupplier A DoubleSupplier that supplies the turn velocity of the robot.
   * @param fieldRelativeSupplier A BooleanSupplier that supplies whether the robot should drive in field relative
   * @param visionAimSupplier A BooleanSupplier that supplies whether the robot should aim using the vision system.
   */
  public TeleopDriveCommand(SwerveSubsystem swerve, VisionSubsystem vision, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier turnSupplier, BooleanSupplier fieldRelativeSupplier, BooleanSupplier visionAimSupplier) {
    this.swerve = swerve;
    this.vision = vision;
    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.turnSupplier = turnSupplier;
    this.fieldRelativeSupplier = fieldRelativeSupplier;
    this.visionAimSupplier = visionAimSupplier;

    addRequirements(this.swerve, this.vision);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get controller inputs
    double forward = MathUtil.applyDeadband(forwardSupplier.getAsDouble() * -1, DriverJoystickConstants.kLeftXDeadband) * swerve.getMaximumDriveVelocity() * 2.5;
    double strafe = MathUtil.applyDeadband(strafeSupplier.getAsDouble() * -1, DriverJoystickConstants.kLeftYDeadband) * swerve.getMaximumDriveVelocity() * 2.5;
    double turn = MathUtil.applyDeadband(turnSupplier.getAsDouble() * -1, DriverJoystickConstants.kRightXDeadband) * DriverJoystickConstants.kTurnMultiplier * swerve.getMaximumAzimuthVelocity() * 2.18;

    boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();
    boolean visionAim = visionAimSupplier.getAsBoolean();

    // If visionAim is true, aim using the vision system
    if (visionAim) {
      var latestResultOptional = vision.getCameras().get(0).getLatestResult(); // TODO how to choose which camera to use?
      if (latestResultOptional.isPresent()) {
        var latestResult = latestResultOptional.get();
        if(latestResult.hasTargets()) {
          var target = latestResult.getBestTarget();
          var yaw = target.bestCameraToTarget.getRotation().getZ();
          turn = -MathUtil.applyDeadband(yaw, VISION_YAW_DEADBAND) * VISION_TURN_kP * swerve.getMaximumAzimuthVelocity();
        }
      }
    }

    swerve.drive(forward, strafe, turn, true, visionAim);
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
