// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.Constants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class CANdleControl extends SubsystemBase {
  CANdle m_candle;
  /** Creates a new CANdleControl. */
  public CANdleControl() {
    m_candle = new CANdle(30); // creates a new CANdle with ID 0
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.05; // dim the LEDs to half brightness
    m_candle.configAllSettings(config);
    m_candle.setLEDs(255, 255, 255); // set the CANdle LEDs to white
    m_candle.clearAnimation(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * sets rgb to a color
   * @param r amount of red
   * @param g amount of green
   * @param b amount of blue
   * @param q starting led number
   * @param c number of total leds
   */
  public void setRGB(int r, int g, int b, int q, int c){
    m_candle.setLEDs(r, g, b, 0, q, c);
  }

  public void setRainbow(){
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 8);
    m_candle.animate(rainbowAnim); 
  }
}