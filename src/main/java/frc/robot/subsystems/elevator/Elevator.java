package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond; 
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotStateManager;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.Constants.RobotStateConstants.ElevatorPosition;

public class Elevator extends SubsystemBase implements ElevatorIO {
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  protected final SparkMax motor = new SparkMax(elevatorMotor1Id, MotorType.kBrushless);
  protected final SparkMax motorFollower = new SparkMax(elevatorMotor2Id, MotorType.kBrushless);
  private final RelativeEncoder encoder, encoderFollower;
  private final SparkClosedLoopController motorController;
  private static final SparkBaseConfig motorConfig = new SparkMaxConfig();
  private static final SparkBaseConfig motorConfigFollower = new SparkMaxConfig();
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(elevatorKs, elevatorKg, elevatorKv, elevatorKa);
  private final SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(Volts.of(0.4).per(Second), Volts.of(1.3), null),
    new SysIdRoutine.Mechanism(this::setVoltage, this::logMotors, this)
  );

  public Elevator() {
    configMotors();

    encoder = motor.getEncoder();
    encoderFollower = motorFollower.getEncoder();

    motorController = motor.getClosedLoopController();

    zeroEncoders();

    elevatorMinSpeed.onChange(this::configMotors);
    elevatorMaxSpeed.onChange(this::configMotors);
    elevatorKp.onChange(this::configMotors);
    elevatorKd.onChange(this::configMotors);
    elevatorPositionTolerance.onChange(this::configMotors);
  }

  /**
   * Configures the motors for the elevator.
   */
  public void configMotors() {
    motorConfig
    .smartCurrentLimit(40)
    .secondaryCurrentLimit(40)
    .idleMode(SparkBaseConfig.IdleMode.kBrake)
    .inverted(elevatorMotorInverted);
    motorConfig.softLimit
    .forwardSoftLimitEnabled(true)
    .forwardSoftLimit(elevatorMaxPosition)
    .reverseSoftLimitEnabled(true)
    .reverseSoftLimit(elevatorMinPosition);
    motorConfig.encoder
    .positionConversionFactor(elevatorDistancePerRevolution)
    .velocityConversionFactor(elevatorVelocityMetersPerSecond);
    motorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(elevatorKp.getAsDouble(), 0.0, elevatorKd.getAsDouble(), elevatorKff)
    .outputRange(elevatorMinSpeed.getAsDouble(), elevatorMaxSpeed.getAsDouble());
    motorConfig.closedLoop.maxMotion
    // .maxVelocity(maxVel)
    // .maxAcceleration(maxAccel)
    .allowedClosedLoopError(elevatorPositionTolerance.getAsDouble());
    motorConfigFollower
    .apply(motorConfig)
    .inverted(elevatorMotorFollowerInverted)
    .follow(motor);
    motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    motorFollower.configure(motorConfigFollower, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);

    try {
      SmartDashboard.putString("Mutables/Elevator Position", RobotStateManager.getDesiredElevatorPosition().toString());
      SmartDashboard.putNumber("Elevator Height", getPosition());
    } catch (Exception e) {}
  }

  /**
   * Sets the speed of the motors.
   *
   * @param speed The speed to set the motors to. Positive values move the
   *     elevator up, negative values move the elevator down.
   */
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  /**
   * Sets the voltage of the motors.
   *
   * @param voltage The voltage to set the motors to in volts. Positive values
   *     move the elevator up, negative values move the elevator down.
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Sets the voltage of the motors.
   *
   * @param voltage The {@link Voltage} to set the motors to in volts. Positive
   *     values move the elevator up, negative values move the elevator down.
   */
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Stops the elevator motors.
   */
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Gets the position of the elevator.
   *
   * @return The position of the elevator.
   */
  public double getPosition() {
    return (encoder.getPosition() + encoderFollower.getPosition()) / 2.0;
  }

  /**
   * Gets the velocity of the elevator.
   *
   * @return The velocity of the elevator.
   */
  public double getVelocity() {
    return (encoder.getVelocity() + encoderFollower.getVelocity()) / 2.0;
  }

  /**
   * Calculate the feedforward voltage for the elevator.
   *
   * @param velocity The velocity setpoint.
   *
   * @return The feedforward voltage as a double.
   */
  public double calculateFeedforward(double velocity) {
    return feedforward.calculate(velocity) *0.5;
  }

  /**
   * Reset the encoder positions of the elevator to zero.
   */
  public void zeroEncoders() {
    encoder.setPosition(0);
  }

  /**
   * Sets the position of the elevator.
   *
   * @param position The position to set the elevator to.
   */
  public void setReference(double position) {
    motorController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateFeedforward(getVelocity()), ArbFFUnits.kVoltage);
  }

  /**
   * Sets the position of the elevator.
   *
   * @param position The {@link ElevatorPosition} to set the elevator to.
   */
  public void setReference(ElevatorPosition position) {
    setReference(position.getAsDouble());
  }

  /**
   * Checks if elevator is at a specific position within a tolerance.
   *
   * @param position The position to check if the elevator is at.
   * @return True if the elevator is at the position, false otherwise.
   */
  public boolean isAtPosition(double position) {
    return Math.abs(getPosition() - position) < elevatorPositionTolerance.getAsDouble();
  }

  /**
   * Checks if elevator is at a specific position within a tolerance.
   * 
   * @param position The {@link ElevatorPosition} to check if the elevator is at.
   * @return True if the elevator is at the position, false otherwise.
   */
  public boolean isAtPosition(ElevatorPosition position) {
    return isAtPosition(position.getAsDouble());
  }

  /**
   * Moves the elevator to a specific position.
   *
   * @param pivot The {@link Pivot} subsystem to check if the pivot is in the way.
   * @param position The supplier of the {@link ElevatorPosition} to move the elevator to.
   * @return The command to move the elevator to the position.
   */
  public Command moveToPosition(Pivot pivot, Supplier<ElevatorPosition> position) {
    return Commands.sequence(
      Commands.runOnce(() -> {
        double targetPosition = position.get().getAsDouble();
  
        setReference(targetPosition);
      }, this)
      .onlyIf(pivot::isOutOfElevatorWay),
      Commands.waitUntil(() -> isAtPosition(position.get()))
    );
  }

  public void logMotors(SysIdRoutineLog log) {
    log.motor("elevator-motor-1").voltage(Volts.of(motor.getBusVoltage() * motor.getAppliedOutput()));
    log.motor("elevator-motor-1").linearPosition(Meters.of(encoder.getPosition()));
    log.motor("elevator-motor-1").linearVelocity(MetersPerSecond.of(encoder.getVelocity()));
    log.motor("elevator-motor-2").voltage(Volts.of(motorFollower.getBusVoltage() * motorFollower.getAppliedOutput()));
    log.motor("elevator-motor-2").linearPosition(Meters.of(encoderFollower.getPosition()));
    log.motor("elevator-motor-2").linearVelocity(MetersPerSecond.of(encoderFollower.getVelocity()));
    }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motorConnected[0] = motor.getFirmwareVersion() != 0;
    inputs.motorConnected[1] = motorFollower.getFirmwareVersion() != 0;
    inputs.motorSpeed[0] = motor.getAppliedOutput();
    inputs.motorSpeed[1] = motorFollower.getAppliedOutput();
    inputs.motorCurrent[0] = motor.getOutputCurrent();
    inputs.motorCurrent[1] = motorFollower.getOutputCurrent();
    inputs.motorVoltage[0] = motor.getBusVoltage();
    inputs.motorVoltage[1] = motorFollower.getBusVoltage();
    inputs.motorTemperature[0] = motor.getMotorTemperature();
    inputs.motorTemperature[1] = motorFollower.getMotorTemperature();
    inputs.motorPosition[0] = encoder.getPosition();
    inputs.motorPosition[1] = encoderFollower.getPosition();
    inputs.motorVelocity[0] = encoder.getVelocity();
    inputs.motorVelocity[1] = encoderFollower.getVelocity();
  }
}
