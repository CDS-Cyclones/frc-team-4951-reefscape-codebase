package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ManipulatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase implements ElevatorIO {
  private final SparkMax motor1 = new SparkMax(elevatorMotor1Id, MotorType.kBrushless);
  private final SparkMax motor2 = new SparkMax(elevatorMotor2Id, MotorType.kBrushless);
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(elevatorKs, elevatorKg, elevatorKv, elevatorKa);
  private final RelativeEncoder encoder1, encoder2;

  private final SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(0.1), null),
    new SysIdRoutine.Mechanism(this::setVoltage, this::logMotors, this)
  );

  public Elevator() {
    encoder1 = motor1.getEncoder();
    encoder2 = motor2.getEncoder();

    motor1.configure(elevatorMotor1Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    motor2.configure(elevatorMotor2Config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    zeroEncoders();
  }

  /**
   * Sets the speed of the motors.
   *
   * @param speed The speed to set the motors to. Positive values move the
   *     elevator up, negative values move the elevator down.
   */
  public void setSpeed(double speed) {
    motor1.set(speed);
    motor2.set(speed);
  }

  /**
   * Sets the voltage of the motors.
   *
   * @param voltage The voltage to set the motors to in volts. Positive values
   *     move the elevator up, negative values move the elevator down.
   */
  public void setVoltage(double voltage) {
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }

  /**
   * Sets the voltage of the motors.
   *
   * @param voltage The {@link Voltage} to set the motors to in volts. Positive
   *     values move the elevator up, negative values move the elevator down.
   */
  public void setVoltage(Voltage voltage) {
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }

  /** Stops the elevator motors. */
  public void stop() {
    motor1.stopMotor();
    motor2.stopMotor();
  }

  /**
   * Gets the position of the elevator.
   *
   * @return The position of the elevator.
   */
  public double getPosition() {
    return (encoder1.getPosition() + encoder2.getPosition()) / 2;
  }

  /**
   * Reset the encoder positions of the elevator to zero.
   */
  public void zeroEncoders() {
    encoder1.setPosition(0);
    encoder2.setPosition(0);
  }

  /**
   * Calculate the feedforward voltage for the elevator.
   *
   * @param velocity The velocity of the elevator.
   * @param acceleration The acceleration of the elevator.
   *
   * @return The feedforward voltage as a double.
   */
  public double calculateFeedforward(double velocity) {
    return feedforward.calculate(velocity);
  }


  public void logMotors(SysIdRoutineLog log) {
    log.motor("elevator-motor-1").voltage(Volts.of(motor1.getBusVoltage() * RobotController.getBatteryVoltage()));
    log.motor("elevator-motor-2").voltage(Volts.of(motor2.getBusVoltage() * RobotController.getBatteryVoltage()));
    }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motorConnected[0] = motor1.getFirmwareVersion() != 0;
    inputs.motorConnected[1] = motor2.getFirmwareVersion() != 0;
    inputs.motorSpeed[0] = motor1.getAppliedOutput();
    inputs.motorSpeed[1] = motor2.getAppliedOutput();
    inputs.motorCurrent[0] = motor1.getOutputCurrent();
    inputs.motorCurrent[1] = motor2.getOutputCurrent();
    inputs.motorVoltage[0] = motor1.getBusVoltage();
    inputs.motorVoltage[1] = motor2.getBusVoltage();
    inputs.motorTemperature[0] = motor1.getMotorTemperature();
    inputs.motorTemperature[1] = motor2.getMotorTemperature();
    inputs.motorWarnings[0] = motor1.getWarnings();
    inputs.motorWarnings[1] = motor2.getWarnings();
    inputs.motorFaults[0] = motor1.getFaults();
    inputs.motorFaults[1] = motor2.getFaults();
    inputs.motorPosition[0] = encoder1.getPosition();
    inputs.motorPosition[1] = encoder2.getPosition();
    inputs.motorVelocity[0] = encoder1.getVelocity();
    inputs.motorVelocity[1] = encoder2.getVelocity();
  }
}
