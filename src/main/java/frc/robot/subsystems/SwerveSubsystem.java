// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import static frc.robot.Constants.SwerveConstants.*;


/**
 * The SwerveSubsystem class is a subsystem that contains the swerve drive of the robot.
 */
public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;

  /////////////////////////////// PID TUNING ///////////////////////////////
  private ShuffleboardTab tab = Shuffleboard.getTab("TunePIDs");

  private GenericEntry driveP = tab.add("drive P", 0).getEntry();
  private GenericEntry driveI = tab.add("drive I", 0).getEntry();
  private GenericEntry driveD = tab.add("drive D", 0).getEntry();
  private GenericEntry driveF = tab.add("drive F", 0).getEntry();

  private GenericEntry angleP = tab.add("angle P", 0).getEntry();
  private GenericEntry angleI = tab.add("angle I", 0).getEntry();
  private GenericEntry angleD = tab.add("angle D", 0).getEntry();
  private GenericEntry angleF = tab.add("angle F", 0).getEntry();
  /////////////////////////////////////////////////////////////////////////

  /**
   * Creates a new SwerveSubsystem.
   */
  public SwerveSubsystem() {
    // Set the verbosity of the telemetry to high (allows to see mroe values on ShuffleBoard)
    SwerveDriveTelemetry.verbosity = TELEMETRY_VERBOSITY;

    // Initialize swerve drive
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setMaximumAllowableSpeeds(MAX_SPEED, MAX_SPEED);

    // Keep the robot heading the same as the previous heading while the robot is translating.
    swerveDrive.setHeadingCorrection(false);

    // Reduces unintended skew in a robot's movement by adjusting angular velocity
    // TODO Tune it by moving the robot in a straight line while rotating and adjusting the compensation coefficient until the deviation is minimized.
    swerveDrive.setAngularVelocityCompensation(false, false, 0.1);

    // Synchronize the absolute encoders and relative encoders everytime bot is at rest for half-a-second
    swerveDrive.setModuleEncoderAutoSynchronize(true, 1);

    // TODO What exactly does this do!?
    swerveDrive.useExternalFeedbackSensor();

    swerveDrive.swerveController.addSlewRateLimiters(null,null, new SlewRateLimiter(1.2));

    // Set motors to brake mode
    setMotorBrake(true);

    try
    {
      // Load PathPlanner config
      // Load the RobotConfig from the GUI settings.
      RobotConfig PPconfig = RobotConfig.fromGUISettings();

      // Configure AutoBuilder (for PathPlanner)
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> swerveDrive.drive(speeds, swerveDrive.kinematics.toSwerveModuleStates(speeds), feedforwards.linearForces()), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
          // Translation PID constants
          new PIDConstants(
            swerveDrive.swerveDriveConfiguration.modules[0].getDrivePIDF().p,
            swerveDrive.swerveDriveConfiguration.modules[0].getDrivePIDF().i,
            swerveDrive.swerveDriveConfiguration.modules[0].getDrivePIDF().d,
            swerveDrive.swerveDriveConfiguration.modules[0].getDrivePIDF().iz
          ),
          // Rotation PID constants
          new PIDConstants(
            swerveDrive.swerveDriveConfiguration.modules[0].getAnglePIDF().p,
            swerveDrive.swerveDriveConfiguration.modules[0].getAnglePIDF().i,
            swerveDrive.swerveDriveConfiguration.modules[0].getAnglePIDF().d,
            swerveDrive.swerveDriveConfiguration.modules[0].getAnglePIDF().iz
          )
        ),
        PPconfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
        },
        this // Reference to this subsystem to set requirements
      );
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }


  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    /////////////////////////////// PID TUNING ///////////////////////////////
    // for(SwerveModule module : swerveDrive.getModules()) {
    //   module.setDrivePIDF(new PIDFConfig(driveP.getDouble(.0020645), driveI.getDouble(0), driveD.getDouble(0), driveF.getDouble(0)));
    //   module.setAnglePIDF(new PIDFConfig(angleP.getDouble(.008), angleI.getDouble(0), angleD.getDouble(.02), angleF.getDouble(0)));
    // }
    /////////////////////////////////////////////////////////////////////////
  }


  /**
   * Drive the robot using translative values and heading as angular velocity.
   *
   * @param velocityX        Translation in the X direction (forward) in m/s.
   * @param velocityY        Translation in the Y direction (left) in m/s.
   * @param angularVelocity  Rotation of the robot to set in rad/s.
   * @param fieldRelative    If true will drive the robot in field relative mode.
   * @param openLoop         If true will use open-loop velocity(higher responsiveness - better for auton), else will use close-loop velocity(higher precision - better for teleop).
   */
  public void drive(double velocityX, double velocityY, double angularVelocity, boolean fieldRelative, boolean openLoop) {
    swerveDrive.drive(new Translation2d(velocityX, velocityY), angularVelocity, fieldRelative, openLoop);
  }

  /** TODO THIS COMMAND ALLOWS TO SWITCH ROTATION POINT
   * Drive the robot using translative values and heading as angular velocity.
   *
   * @param velocityX        Translation in the X direction (forward) in m/s.
   * @param velocityY        Translation in the Y direction (left) in m/s.
   * @param angularVelocity  Rotation of the robot to set in rad/s.
   */
  public void drive(double velocityX, double velocityY, double angularVelocity, Translation2d rotationPoint) {
    swerveDrive.drive(new ChassisSpeeds(velocityX, velocityY, angularVelocity), false, rotationPoint);
  }


  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }


  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Resets odometry to right cornerfacing forward. Gyro angle and module positions do not need to be reset when calling this
   * method.
   */
  public void resetOdometry() {
    swerveDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
  }


  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }


  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }


  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }


  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }


  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }


  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   *
   * If red alliance rotate the robot 180 after the drivebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }


  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }


  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }


  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX, headingY, getHeading().getRadians(), MAX_SPEED);
  }


  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), angle.getRadians(), getHeading().getRadians(), MAX_SPEED);
  }


  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }


  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }


  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }


  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }


  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }


  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }


  /**
   * Return the maximum translational velocity of chassis.
   *
   * @return maximum translational velocity of chassis in m/s.
   */
  public double getMaximumDriveVelocity() {
    return swerveDrive.getMaximumChassisVelocity();
  }


  /**
   * Return the maximum rotational velocity of chassis.
   *
   * @return maximum rotational velocity of chassis as a double in rad/s.
   */
  public double getMaximumAzimuthVelocity() {
    return swerveDrive.getMaximumChassisAngularVelocity();
  }


  /**
   * Update odometry. Should be run every loop.
   */
  public void updateOdometry() {
    swerveDrive.updateOdometry();
  }


  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the SwerveIMU gyro reading with the given timestamp of the vision measurement.
   * 
   * @param estimatedPose Robot {@link Pose2d} as measured by vision.
   * @param timestampSecondsTimestamp Time since startup.
   * @param estimationStdDevs Vision measurement standard deviation that will be sent to the {@link SwerveDrivePoseEstimator}.
   */
  public void addVisionMeasurement(Pose2d estimatedPose, double timestampSeconds, Matrix<N3, N1> estimationStdDevs) {
    swerveDrive.addVisionMeasurement(estimatedPose, timestampSeconds, estimationStdDevs);
  }


  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true), 3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 5.0, 3.0);
  }
}
