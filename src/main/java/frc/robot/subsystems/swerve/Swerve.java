// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.LimelightHelpers;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class Swerve extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset
  );

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset
  );

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset
  );

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset
  );

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroCanId, DriveConstants.kGyroCanBusName);


  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    m_gyro.getRotation2d(),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    },
    new Pose2d(),
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
  );

  /** Creates a new DriveSubsystem. */
  public Swerve() throws IOException, ParseException {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    zeroHeading();

    // Load the RobotConfig from the GUI settings.
    RobotConfig pathplannerConfig = RobotConfig.fromGUISettings();

    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
      new PPHolonomicDriveController( // PPHolonomicController is the built-in path following controller for holonomic drive trains
        new PIDConstants(5, 0, 0, 0),
        new PIDConstants(0.5, 0, 0, 0)
      ),
      pathplannerConfig, // PathPlanner config from GUI
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
      },
      this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_poseEstimator.update(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      }
    );

    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation(Constants.LimelightConstants.kLimelightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimelightConstants.kLimelightName);
    if (Math.abs(getTurnRate())> 720) { // if ang vel is > 2 rev/sec, ignore vision updates
      doRejectUpdate = true;
    }
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
      Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      pose
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *     the
   *                      field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeedDelivered,
        ySpeedDelivered,
        rotDelivered,
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble())
      )
      : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

    /**
     * Method to drive the robot using chassis speeds in robot-relative coordinates.
     *
     * @param speeds The speeds to drive at.
     */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble())), 0.2);
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0);
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().refresh().getValueAsDouble()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double gyroRateZ = m_gyro.getAngularVelocityZDevice().refresh().getValueAsDouble();

    return gyroRateZ;
  }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds of the robot.
     */
  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

    /**
     * Returns the current wheel states of the robot.
     *
     * @return The current wheel states of the robot.
     */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    return states;
  }

    /**
     * Returns the current pose of the robot.
     *
     * @return The current pose of the robot.
     */
    public Pose2d getEstimatedPosition() {
        return m_poseEstimator.getEstimatedPosition();
    }
}
