// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.VISION_TURN_kP;
import static frc.robot.Constants.VisionConstants.VISION_YAW_DEADBAND;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverJoystickConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends Command {
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final DoubleSupplier forwardSupplier, strafeSupplier, turnSupplier;
  private final BooleanSupplier fieldRelativeSupplier, visionAimSupplier;


  /** Creates a new TeleopDriveCommand.
   *
   * @param swerve The SwerveSubsystem used by this command to drive the robot.
   * @param forwardSupplier A DoubleSupplier that supplies the forward velocity of the robot.
   * @param strafeSupplier A DoubleSupplier that supplies the strafe velocity of the robot.
   * @param turnSupplier A DoubleSupplier that supplies the turn velocity of the robot.
   * @param fieldRelativeSupplier A BooleanSupplier that supplies whether the robot should drive in field relative
   * @param visionAimSupplier A BooleanSupplier that supplies whether the robot should aim using the vision system.
   */
  public TeleopDriveCommand(SwerveSubsystem swerve, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier turnSupplier, BooleanSupplier fieldRelativeSupplier, BooleanSupplier visionAimSupplier) {
    this.swerve = swerve;
    this.vision = swerve.getVisionSubsystem();
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
    double forward = MathUtil.applyDeadband(forwardSupplier.getAsDouble() * -1, DriverJoystickConstants.kLeftXDeadband) * swerve.getMaximumDriveVelocity();
    double strafe = MathUtil.applyDeadband(strafeSupplier.getAsDouble() * -1, DriverJoystickConstants.kLeftYDeadband) * swerve.getMaximumDriveVelocity();
    double turn = MathUtil.applyDeadband(turnSupplier.getAsDouble() * -1, DriverJoystickConstants.kRightXDeadband) * DriverJoystickConstants.kTurnMultiplier * swerve.getMaximumAzimuthVelocity();
    boolean fieldRelative = !fieldRelativeSupplier.getAsBoolean();
    boolean visionAim = visionAimSupplier.getAsBoolean();

    // If visionAim is true, aim using the vision system
    if (visionAim) {
      var latestResultOptional = vision.getLatestResult();
      if (latestResultOptional.isPresent()) {
        var latestResult = latestResultOptional.get();
        if(latestResult.hasTargets()) {
          var target = latestResult.getBestTarget();
          turn = MathUtil.applyDeadband(target.getYaw(), VISION_YAW_DEADBAND) * VISION_TURN_kP * swerve.getMaximumAzimuthVelocity();
        }
      }
    }

    // Drive the robot
    swerve.drive(forward, strafe, turn, fieldRelative, visionAim);
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
