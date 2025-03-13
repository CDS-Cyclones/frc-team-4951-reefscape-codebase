// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.*;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.mutables.MutableCandleState;
import frc.robot.mutables.MutableCandleState.CandleState;
import static frc.robot.Constants.CandleConstants.*;


public class Candle extends SubsystemBase implements CandleIO {
  private final CANdle candle = new CANdle(candleId, candleBus);
  private final CandleIOInputsAutoLogged candleInputs = new CandleIOInputsAutoLogged();

  public Candle() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = candleBrightness;
    configAll.vBatOutputMode = VBatOutputMode.Off;
    candle.configAllSettings(configAll, 100);
  }

  /* Wrappers so we can access the CANdle from the subsystem */
  public double getVbat() {
    return candle.getBusVoltage();
  }
  public double get5V() {
    return candle.get5VRailVoltage();
  }
  public double getCurrent() {
    return candle.getCurrent();
  }
  public double getTemperature() {
    return candle.getTemperature();
  }
  public void configBrightness(double percent) {
    candle.configBrightnessScalar(percent, 0);
  }
  public void configLos(boolean disableWhenLos) {
    candle.configLOSBehavior(disableWhenLos, 0);
  }
  public void configLedType(LEDStripType type) {
    candle.configLEDType(type, 0);
  }
  public void configStatusLedBehavior(boolean offWhenActive) {
    candle.configStatusLedState(offWhenActive, 0);
  }

  public void setLEDs(int red, int green, int blue) {
    // Clamp the values to 0-255
    red = Math.max(0, Math.min(255, red));
    green = Math.max(0, Math.min(255, green));
    blue = Math.max(0, Math.min(255, blue));

    candle.setLEDs(red, green, blue);
  }

  @Override
  public void periodic() {
    CandleState state = MutableCandleState.getMutableCandleState();

    setLEDs(state.getRed(), state.getGreen(), state.getBlue());

    updateInputs(candleInputs);
    Logger.processInputs("Candle", candleInputs);
  }

  @Override
  public void updateInputs(CandleIOInputs inputs) {
    inputs.temperature = getTemperature();
    inputs.busVoltage = getVbat();
    inputs.railVoltage = get5V();
    inputs.current = getCurrent();
    inputs.brightness = candle.configGetParameter(ParamEnum.eBrightnessCoefficient, 0);
    inputs.state = MutableCandleState.getMutableCandleState();
  }
}