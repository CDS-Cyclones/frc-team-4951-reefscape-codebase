
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import static frc.robot.Constants.VisionConstants.OMEGA_CONSTRAINTS;
import static frc.robot.Constants.VisionConstants.OMEGA_PID_CONSTANTS;
import static frc.robot.Constants.VisionConstants.OMEGA_TOLERANCE;
import static frc.robot.Constants.VisionConstants.X_CONSTRAINTS;
import static frc.robot.Constants.VisionConstants.X_PID_CONSTANTS;
import static frc.robot.Constants.VisionConstants.X_TOLERANCE;
import static frc.robot.Constants.VisionConstants.Y_CONSTRAINTS;
import static frc.robot.Constants.VisionConstants.Y_PID_CONSTANTS;
import static frc.robot.Constants.VisionConstants.Y_TOLERANCE;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants.PoseRelToAprilTag;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** A command that chases a target using the swerve drive.
 * This command uses the photon camera to track a target and then uses a PID controller to drive the robot to the target.
 * The {@link ProfiledPIDController}s are used to control the x, y, and omega (rotation) of the robot.
 * The command ends  the robot reaches the target pose.
 */
public class ChaseTagCommand extends Command {
  private final VisionSubsystem vision;
  private final SwerveSubsystem swerve;
  private final Supplier<Pose2d> poseSupplier;
  private final PoseRelToAprilTag desiredPose;

  private final ProfiledPIDController xController = new ProfiledPIDController(X_PID_CONSTANTS.kP, X_PID_CONSTANTS.kI, X_PID_CONSTANTS.kD, X_CONSTRAINTS);  
  private final ProfiledPIDController yController = new ProfiledPIDController(Y_PID_CONSTANTS.kP, Y_PID_CONSTANTS.kI, Y_PID_CONSTANTS.kD, Y_CONSTRAINTS);  
  private final ProfiledPIDController omegaController = new ProfiledPIDController(OMEGA_PID_CONSTANTS.kP, OMEGA_PID_CONSTANTS.kI, OMEGA_PID_CONSTANTS.kD, OMEGA_CONSTRAINTS);  

  private PhotonTrackedTarget latestTarget;

  /////////////////////////////// PID TUNING ///////////////////////////////
  private ShuffleboardTab tab = Shuffleboard.getTab("TunePIDs");

  private GenericEntry xP = tab.add("x P", 0).getEntry();
  private GenericEntry xI = tab.add("x I", 0).getEntry();
  private GenericEntry xD = tab.add("x D", 0).getEntry();
  private GenericEntry yP = tab.add("y P", 0).getEntry();
  private GenericEntry yI = tab.add("y I", 0).getEntry();
  private GenericEntry yD = tab.add("y D", 0).getEntry();
  private GenericEntry omegaP = tab.add("omega P", 0).getEntry();
  private GenericEntry omegaI = tab.add("omega I", 0).getEntry();
  private GenericEntry omegaD = tab.add("omega D", 0).getEntry();
  /////////////////////////////////////////////////////////////////////////


  /** Creates a new ChaseTagCommand. */
  public ChaseTagCommand(VisionSubsystem vision, SwerveSubsystem swerve, Supplier<Pose2d> poseSupplier, PoseRelToAprilTag desiredPose) {
    this.vision = vision;
    this.swerve = swerve;
    this.poseSupplier = poseSupplier;
    this.desiredPose = desiredPose;

    xController.setTolerance(X_TOLERANCE);
    yController.setTolerance(Y_TOLERANCE);
    omegaController.setTolerance(OMEGA_TOLERANCE);
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(this.vision, this.vision.getCameras().get(0), this.swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    latestTarget = null;
    var robotPose = poseSupplier.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());

    /////////////////////////////// PID TUNING ///////////////////////////////
    xController.setP(xP.getDouble(0));
    xController.setI(xI.getDouble(0));
    xController.setD(xD.getDouble(0));  
    yController.setP(yP.getDouble(0));
    yController.setI(yI.getDouble(0));
    yController.setD(yD.getDouble(0));
    omegaController.setP(omegaP.getDouble(0));
    omegaController.setI(omegaI.getDouble(0));
    omegaController.setD(omegaD.getDouble(0));
    /////////////////////////////////////////////////////////////////////////
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose2d = poseSupplier.get();
    var robotPose = new Pose3d(robotPose2d);

    //for (PVCamera cam : vision.getCameras()) {  
      var photonResultOptional = vision.getCameras().get(0).getLatestResult();
      if(photonResultOptional.isPresent()) {
        System.out.println("hjkjfhgueifkeyuefefe");
        var photonResult = photonResultOptional.get();
        if(photonResult.hasTargets()) {
          // var targetOptional = photonResult.getTargets().stream().filter(t -> t.getFiducialId() == desiredPose.aprilTagId).filter(t -> t.getPoseAmbiguity() <= 2).findFirst();
          var   betstrget = photonResult.getBestTarget();
          if(true) {
            System.out.println(betstrget.getFiducialId());
            latestTarget =betstrget;
            var goalPose = robotPose.transformBy(vision.getCameras().get(0).getBotToCam()).transformBy(latestTarget.getBestCameraToTarget()).transformBy(desiredPose.relativePose).toPose2d();
            
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());
          }
        }
      }
    // }
  

    if(latestTarget == null) {
      swerve.drive(0, 0, 0, false, true);
    } else {
      var xSpeed = xController.calculate(robotPose.getX());
      if(xController.atGoal()) xSpeed = 0;

      var ySpeed = yController.calculate(robotPose.getY());
      if(yController.atGoal()) ySpeed = 0;

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if(omegaController.atGoal()) omegaSpeed = 0;

      swerve.drive(xSpeed, ySpeed, omegaSpeed, false, true);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
  }
}
