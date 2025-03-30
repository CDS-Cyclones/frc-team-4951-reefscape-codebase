// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.*;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants.CandleState;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.CandleConstants.*;


public class Candle extends SubsystemBase implements CandleIO {
  private final CANdle candle = new CANdle(candleId, candleBus);
  private final CandleIOInputsAutoLogged candleInputs = new CandleIOInputsAutoLogged();

  // private final AddressableLED ledStrip = new AddressableLED(ledPWMPort);
  // private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledCount);

  private final Spark blinking = new Spark(ledPWMPort);

  @Setter @Getter private CandleState state = CandleState.OFF;

  public Candle() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
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

  public void setLEDs(int r, int g, int b, double blinkingPwmValue) {
    // Clamp the values to 0-255
    r = Math.max(0, Math.min(255, r));
    g = Math.max(0, Math.min(255, g));
    b = Math.max(0, Math.min(255, b));

    candle.setLEDs(r, g, b);
    blinking.set(blinkingPwmValue);
  }

  public void setLEDs(CandleState state) {
    // Clamp the values to 0-255
    int r = Math.max(0, Math.min(255, state.getRed()));
    int g = Math.max(0, Math.min(255, state.getGreen()));
    int b = Math.max(0, Math.min(255, state.getBlue()));
    double blinkingPwmValue = state.getBlinkingPwmValue();

    setLEDs(r, g, b, blinkingPwmValue);
  }

  @Override
  public void periodic() {
    updateInputs(candleInputs);
    Logger.processInputs("Candle", candleInputs);

    try {
    SmartDashboard.putString("Mutables/Candle State", state.toString());
    } catch (Exception e) {}
  }

  @Override
  public void updateInputs(CandleIOInputs inputs) {
    inputs.temperature = getTemperature();
    inputs.busVoltage = getVbat();
    inputs.railVoltage = get5V();
    inputs.current = getCurrent();
    inputs.brightness = candle.configGetParameter(ParamEnum.eBrightnessCoefficient, 0);
    inputs.state = state;
  }
}