package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotStateConstants.ElevatorPosition;
import frc.robot.Constants.RobotStateConstants.PivotPosition;
import frc.robot.RobotStateManager;

public class Pivot extends SubsystemBase implements PivotIO {
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final SparkMax motor = new SparkMax(pivotMotorId, MotorType.kBrushless);
  private final AbsoluteEncoder absoluteEncoder;
  private final RelativeEncoder relativeEncoder;
  private final SparkClosedLoopController motorController;
  private static final SparkBaseConfig motorConfig = new SparkMaxConfig();
  private final ArmFeedforward feedforward = new ArmFeedforward(pivotKs, pivotKg, pivotKv, pivotKa);
  private final SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(0.1), null),
    new SysIdRoutine.Mechanism(this::setVoltage, this::logMotors, this)
  );

  public Pivot() {
    configMotor();

    absoluteEncoder = motor.getAbsoluteEncoder();
    relativeEncoder = motor.getEncoder();
    motorController = motor.getClosedLoopController();

    // Set point where pivot is not pushed either way by gravity as 0
    relativeEncoder.setPosition(absoluteEncoder.getPosition() + pivotOffsetFromEquilibrium);

    pivotKp.onChange(this::configMotor);
    pivotKd.onChange(this::configMotor);
    pivotMinSpeed.onChange(this::configMotor);
    pivotMaxSpeed.onChange(this::configMotor);
    pivotPositionTolerance.onChange(this::configMotor);
  }

  /**
   * Configures the motor for the pivot.
   */
  public void configMotor() {
    motorConfig
    .smartCurrentLimit(80)
    .secondaryCurrentLimit(90)
    .idleMode(SparkBaseConfig.IdleMode.kBrake)
    .inverted(pivotMotorInverted);
    motorConfig.softLimit
    .forwardSoftLimitEnabled(true)
    .forwardSoftLimit(pivotMaxPosition)
    .reverseSoftLimitEnabled(true)
    .reverseSoftLimit(pivotMinPosition);
    motorConfig.absoluteEncoder
    .positionConversionFactor(pivotAbsoluteEncoderRadiansPerRevolution)
    .velocityConversionFactor(pivotAbsoluteEncoderAngularVelocityRadiansPerSecond)
    .inverted(pivotAbsoluteEncoderInverted);
    motorConfig.encoder
    .positionConversionFactor(pivotRelativeEncoderRadiansPerRevolution)
    .velocityConversionFactor(pivotRelativeEncoderAngularVelocityRadiansPerSecond);
    motorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(pivotKp.getAsDouble(), 0.0, pivotKd.getAsDouble(), pivotKff)
    .outputRange(pivotMinSpeed.getAsDouble(), pivotMaxSpeed.getAsDouble());
    motorConfig.closedLoop.maxMotion
    // .maxVelocity(maxVel)
    // .maxAcceleration(maxAccel)
    .allowedClosedLoopError(pivotPositionTolerance.getAsDouble());
    motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }


  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // Set point where pivot is not pushed either way by gravity as 0
    relativeEncoder.setPosition(absoluteEncoder.getPosition() + pivotOffsetFromEquilibrium);

    updateInputs(pivotInputs);
    Logger.processInputs("Pivot", pivotInputs);

    try {
      SmartDashboard.putString("Mutables/Pivot Position", RobotStateManager.getDesiredPivotPosition().toString());
      SmartDashboard.putNumber("Pivot Position", getPosition());
      SmartDashboard.putNumber("Absolute Pivot Position", absoluteEncoder.getPosition());
      SmartDashboard.putBoolean("Pivot out of the way", isOutOfElevatorWay());
    } catch (Exception e) {}
  }

  /**
   * Sets the speed of the pivot motor.
   *
   * @param speed The speed to set the motor to. Positive values move the pivot out, negative values move the arm in.
   */
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  /**
   * Sets the voltage of the pivot motor.
   *
   * @param voltage The {@link Voltage} to set the motor to in volts. Positive values move the pivot out, negative values move the arm in.
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Sets the voltage of the pivot motor.
   *
   * @param voltage The {@link Voltage} to set the motor to in volts. Positive values move the pivot out, negative values move the arm in.
   */
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Stops the pivot motor.
   */
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Gets the position of the pivot.
   *
   * @return The position of the pivot.
   */
  public double getPosition() {
    return relativeEncoder.getPosition();
  }

  /**
   * Gets the velocity of the pivot.
   *
   * @return The velocity of the pivot.
   */
  public double getVelocity() {
    return relativeEncoder.getVelocity();
  }

  /**
   * Calculate the feedforward voltage for the pivot.
   *
   * @param positionRadians The position (angle) setpoint. This angle should be measured from the
   * horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). 
   * @param velocity The velocity setpoint.
   *
   * @return The feedforward voltage as a double.
   */
  public double calculateFeedforward() {
    return feedforward.calculate(getPosition(), getVelocity());
  }

  /**
   * Checks if the pivot will not be in elevator's way
   *
   * @return True if no tin the way, false otherwise
   */
  public boolean isOutOfElevatorWay() {
    return getPosition() >= pivotMinPositionForElevatorMovement;
  }

    /**
   * Sets the position of the pivot.
   *
   * @param position The position to set the elevator to.
   */
  public void setReference(double position) {
    motorController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateFeedforward(), ArbFFUnits.kVoltage);
  }

  /**
   * Sets the position of the pivot.
   *
   * @param position The {@link ElevatorPosition} to set the elevator to.
   */
  public void setReference(PivotPosition position) {
    setReference(position.getAsDouble());
  }

  /**
   * Checks if pivot is at a specific position within a tolerance.
   *
   * @param position The position to check if the elevator is at.
   * @return True if the elevator is at the position, false otherwise.
   */
  public boolean isAtPosition(double position) {
    return Math.abs(getPosition() - position) < elevatorPositionTolerance.getAsDouble();
  }

  /**
   * Checks if pivot is at a specific position within a tolerance.
   * 
   * @param position The {@link ElevatorPosition} to check if the elevator is at.
   * @return True if the elevator is at the position, false otherwise.
   */
  public boolean isAtPosition(PivotPosition position) {
    return isAtPosition(position.getAsDouble());
  }

  /**
   * Moves the pivot to a specific position.
   *
   * @param position The {@link PivotPosition} to move the pivot to.
   * @param position The {@link PivotPosition} to move the pivot to.
   * @return The command to move the pivot to the position.
   */
  public Command moveToPosition(Supplier<PivotPosition> position) {
    return Commands.run(() -> {
      double targetPosition = position.get().getAsDouble();

      setReference(targetPosition);
    }, this)
    .until(() -> 
      isAtPosition(position.get())
    );
  }

  public void logMotors(SysIdRoutineLog log) {
    log.motor("pivot-motor").voltage(Volts.of(motor.getBusVoltage() * RobotController.getBatteryVoltage()));
    log.motor("pivot-motor").angularPosition(Radians.of(getPosition()));
    log.motor("pivot-motor").angularVelocity(RadiansPerSecond.of(getVelocity()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.motorConnected = motor.getFirmwareVersion() != 0;
    inputs.motorSpeed = motor.getAppliedOutput();
    inputs.motorCurrent = motor.getOutputCurrent();
    inputs.motorVoltage = motor.getBusVoltage();
    inputs.motorTemperature = motor.getMotorTemperature();
    inputs.motorAbsolutePosition = absoluteEncoder.getPosition();
    inputs.motorRelativePosition = relativeEncoder.getPosition();
    inputs.motorVelocity = relativeEncoder.getVelocity();
  }
}
