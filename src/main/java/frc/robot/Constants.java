// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.TunableValues.TunableNum;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(27.0);  // TODO measure
    public static final double wheelBase = Units.inchesToMeters(32.5);  // TODO measure
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2.0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);

    // Device CAN IDs
    public static final int pigeon2CanId = 9;
    public static final String pigeon2CanBus = "pigeonbus";

    public static final int frontLeftDriveCanId = 21;
    public static final int backLeftDriveCanId = 27;
    public static final int frontRightDriveCanId = 23;
    public static final int backRightDriveCanId = 25;

    public static final int frontLeftTurnCanId = 22;
    public static final int backLeftTurnCanId = 28;
    public static final int frontRightTurnCanId = 24;
    public static final int backRightTurnCanId = 26;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 80;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);  // TODO Run Drive Wheel Radius Characterization
    public static final double driveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final TunableNum driveKp = new TunableNum("Drive/drive/p", 0.0);  // TODO tune
    public static final TunableNum driveKd = new TunableNum("Drive/drive/d", 0.0);  // TODO tune
    public static final double driveKs = 0.0;  // TODO Run Drive Wheel Radius Characterization
    public static final double driveKv = 0.0;  // TODO Run Drive Wheel Radius Characterization
    public static final double driveSimP = 0.0;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.04038;
    public static final double driveSimKv = 0.11972;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 40;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final TunableNum turnKp = new TunableNum("Drive/turn/p", 2.0);  // TODO tune
    public static final TunableNum turnKd = new TunableNum("Drive/turn/d", 0.1);  // TODO tune
    public static final double turnSimP = 10;
    public static final double turnSimD = 0.2;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // Configuration for PID controllers
    public static final TunableNum anglePIDCKp = new TunableNum("Drive/PIDController/angle/p", 6.0);  // TODO tune
    public static final TunableNum anglePIDCKd = new TunableNum("Drive/PIDController/angle/d", 0.4);  // TODO tune
    public static final TunableNum anglePIDCMaxVel = new TunableNum("Drive/PIDController/angle/maxVel", 8);  // TODO tune // Radians per second
    public static final TunableNum anglePIDCMaxAccel = new TunableNum("Drive/PIDController/angle/maxAccel", 20);  // TODO tune // Radians per second squared
    public static final TunableNum anglePIDTolerance = new TunableNum("Drive/PIDController/angle/errorTolerance", 0.05);  // TODO tune
    public static final TunableNum translationPIDCKp = new TunableNum("Drive/PIDController/translation/p", 4);  // TODO tune
    public static final TunableNum translationPIDCKd = new TunableNum("Drive/PIDController/translation/d", 0.1);  // TODO tune
    public static final TunableNum translationPIDCMaxVel = new TunableNum("Drive/PIDController/translation/maxVel", 6);  // TODO tune // Meters per second
    public static final TunableNum translationPIDCMaxAccel = new TunableNum("Drive/PIDController/translation/maxAccel", 15);  // TODO tune // Meters per second squared
    public static final TunableNum translationPIDTolerance = new TunableNum("Drive/PIDController/translation/errorTolerance", 0.05);  // TODO tune

    // PathPlanner configuration
    public static final double robotMassKg = 52;  // TODO measure
    public static final double robotMOI = 6.883;  // TODO calculate w/ formula after running SysId
    public static final double wheelCOF = 1.2;
    public static final PIDConstants ppDrivePID = new PIDConstants(5.0, 0.0, 0.0);  // TODO tune
    public static final PIDConstants ppTurnPID = new PIDConstants(5.0, 0.0, 0.0);  // TODO tune
    public static final RobotConfig ppConfig =
      new RobotConfig(
        robotMassKg,
        robotMOI,
        new ModuleConfig(
          wheelRadiusMeters,
          maxSpeedMetersPerSec,
          wheelCOF,
          driveGearbox.withReduction(driveMotorReduction),
          driveMotorCurrentLimit,
          1
        ),
        moduleTranslations
      );
    
    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
      .withCustomModuleTranslations(moduleTranslations)
      .withRobotMass(Kilograms.of(robotMassKg))
      .withGyro(COTS.ofPigeon2())
      .withSwerveModule(new SwerveModuleSimulationConfig(
        driveGearbox,
        turnGearbox,
        driveMotorReduction,
        turnMotorReduction,
        Volts.of(0.1),
        Volts.of(0.1),
        Meters.of(wheelRadiusMeters),
        KilogramSquareMeters.of(robotMOI),
        wheelCOF
      )
    );
  }

  public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static String cameraName = "limelight";
    public static String cameraNameSim = "simCam";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d botToCamTransformSim = new Transform3d(
      new Translation3d(-.3, 0, 0), // X is forward in m, z is up in m
      new Rotation3d(0, 0, 0)  // facing forward
    );
    
    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {1.0};

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class OIConstants {
    // Joystick ports
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorBoardPort = 1;
    public static final int kOperatorControllerPort = 2;
    public static final int kOperatorControllerManualPort = 3;

    // Joystick axis deadband
    public static final double kJoystickAxisDeadband = 0.1;
  }

  public static final class ManipulatorConstants {
    // CAN IDs
    public static final int elevatorMotor1Id = 31;
    public static final int elevatorMotor2Id = 32;
    public static final int pivotMotorId = 56;
    public static final int intakeMotorId = 57;
    public static final int canrangeCanId = 40;
    public static final String canrangeCanBus = "pigeonbus";

    // Motor configurations
    public static final SparkBaseConfig elevatorMotor1Config = new SparkMaxConfig()
      .smartCurrentLimit(80)
      .secondaryCurrentLimit(90)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(false);
    public static final SparkBaseConfig elevatorMotor2Config = new SparkMaxConfig()
      .smartCurrentLimit(80)
      .secondaryCurrentLimit(90)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(false);
    public static final SparkBaseConfig pivotMotorConfig = new SparkMaxConfig()
      .smartCurrentLimit(80)
      .secondaryCurrentLimit(90)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(true);
    public static final SparkBaseConfig intakeWheelsMotorConfig = new SparkMaxConfig()
      .smartCurrentLimit(80)
      .secondaryCurrentLimit(90)
      .idleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(false);

    // Elevator constants
    public static final double elevatorMinPosition = 0.0;  // TODO figure out
    public static final double elevatorMaxPosition = 1.0;  // TODO figure out
    public static TunableNum elevatorMaxSpeed = new TunableNum("Elevator/MaxSpeed", 0.5);  // TODO tune
    public static final TunableNum elevatorMaxAcceleration = new TunableNum("Elevator/MaxAcceleration", 0.5);  // TODO tune
    public static final TunableNum elevatorKp = new TunableNum("Elevator/P", 0.5);
    public static final TunableNum elevatorKd = new TunableNum("Elevator/D", 0.0);
    public static final double elevatorKs = 0.0;  // TODO figure out
    public static final double elevatorKg = 0.0;  // TODO figure out
    public static final double elevatorKv = 0.0;  // TODO figure out
    public static final double elevatorKa = 0.0;  // TODO figure out
    public static final TunableNum elevatorPIDTolerance =new TunableNum("Elevator/ErrorTolerance", 0.01);  // TODO tune

    // Pivot constants
    public static final double pivotMinPosition = 0.0;  // TODO figure out
    public static final double pivotMaxPosition = 1.0;  // TODO figure out
    public static final TunableNum pivotMaxSpeed = new TunableNum("Pivot/MaxSpeed", 0.5);  // TODO tune
    public static final TunableNum pivotMaxAcceleration = new TunableNum("Pivot/MaxAcceleration", 0.5);  // TODO tune
    public static final TunableNum pivotKp = new TunableNum("Pivot/P", 0.5);  // TODO tune
    public static final TunableNum pivotKd = new TunableNum("Pivot/D", 0.0);  // TODO tune
    public static final double pivotKs = 0.0;  // TODO figure out
    public static final double pivotKg = 0.0;  // TODO figure out
    public static final double pivotKv = 0.0;  // TODO figure out
    public static final double pivotKa = 0.0;  // TODO figure out
    public static final TunableNum pivotPIDTolerance = new TunableNum("Pivot/ErrorTolerance", 0.0);  // TODO tune
    public static final double pivotMinPositionForElevatorMovement = 10000000;  // TODO figure out

    public static final double intakeRangeSensorThreshold = 0.05; // in m TODO figure this out
    public static final TunableNum coralIntakeSpeed = new TunableNum("Intake/CoralIntakeSpeed", 0.2);  // TODO tune
    public static final TunableNum coralScoringTime = new TunableNum("Intake/CoralScoringTime", 0.5);  // TODO tune // in sec
    public static final TunableNum coralScoringSpeed = new TunableNum("Intake/CoralScoringSpeed", 0.2);  // TODO tune
  }

  public static class CANdleConstants {
    public static final int CANid = 40;
    public static final String CANbus = "pigeonbus";
  }
}
