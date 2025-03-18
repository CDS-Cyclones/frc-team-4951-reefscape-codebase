package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ManipulatorConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotStateManager;

public class Pivot extends SubsystemBase implements PivotIO {
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final SparkMax motor = new SparkMax(pivotMotorId, MotorType.kBrushless);
  private final ArmFeedforward feedforward = new ArmFeedforward(pivotKs, pivotKg, pivotKv, pivotKa);
  private final AbsoluteEncoder encoder;
  double blah = 0;

  private final SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(0.1), null),
    new SysIdRoutine.Mechanism(this::setVoltage, this::logMotors, this)
  );

  public Pivot() {
    encoder = motor.getAbsoluteEncoder();
  
    motor.configure(pivotMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateInputs(pivotInputs);
    Logger.processInputs("Pivot", pivotInputs);

    try {
      SmartDashboard.putString("Mutables/Pivot Position", RobotStateManager.getDesiredPivotPosition().toString());
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
   * Gets the absolute position of the pivot.
   *
   * @return The absolute position of the pivot.
   */
  public double getPosition() {
    return encoder.getPosition();
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
  public double calculateFeedforward(double velocity, double acceleration) {
    return feedforward.calculate(velocity, acceleration);
  }

  /**
   * Checks if the pivot will not be in elevator's way going up.
   *
   * @return True if no tin the way, false otherwise
   */
  public boolean isOutOfElevatorWay() {
    return getPosition() >= pivotMinPositionForElevatorMovement;
  }

  public void logMotors(SysIdRoutineLog log) {
    log.motor("pivot-motor-1").voltage(Volts.of(motor.getBusVoltage() * RobotController.getBatteryVoltage()));
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
    inputs.motorAbsolutePosition = encoder.getPosition();
    inputs.motorVelocity = encoder.getVelocity();
  }
}
