// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Bling extends SubsystemBase {
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;
  public AprilTagFinder aprilTagFinder;

  public int eyesLength = 48;
  public int eyesQuarterLength = 6;
  public int stripsLength = 32;
  public int stripSlotLength = 16;

  
  /**
   * Creates a new bling.
   * @param key - the key name
   * @param defaultValue - the value to be returned if no value is found
   * @param length - the strip length
   * @param buffer - the buffer to write
   * @return Global default instance
   * @return The network table
   * @return Nework table entry
   * @return the entry's value or the given default value
   * @return the buffer length
   */
  public Bling(AprilTagFinder aprilTagFinder) {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(eyesLength + stripsLength);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    // m_ledArms = new AddressableLED(1);
    // m_ledBufferArms = new AddressableLEDBuffer(armsLength);
    // m_ledArms.setLength(m_ledBufferArms.getLength());
    // m_ledArms.setData(m_ledBufferArms);
    // m_ledArms.start();

    this.aprilTagFinder = aprilTagFinder;
  }

  /**
   * Clears all of the LEDs on the robot.
   */
  public void initialize() {
    clearLEDs();

  }

  /**
   * Sets the buffer and runs battery bling.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);
    setBatteryBling();

    if(!DriverStation.isDisabled()){
      setBatteryBling();
    }
  }

  /**
   * Sets one LED to a color
   * @param i - the index to write
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   */
  public void setRGB(int i, int r, int g, int b)
  {
    m_ledBuffer.setRGB(i, r, g, b);
  }

  /**
   * Sets all the LEDs to one color
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   * @return buffer length
   */
  public void setEyesRGBAll(int r, int g, int b) {
    for (var i = 0; i < eyesLength; i++) {
      setRGB(i, r, g, b);
    }
  }

  public void setStripRGBAll(int r, int g, int b){
    for (var i = eyesLength; i < m_ledBuffer.getLength(); i++){
      setRGB(i, r, g, b);
    }
  }

  /**
   * Turns off all LEDs
   */
  public void clearLEDs() {
    setEyesRGBAll(0, 0, 0);
    setStripRGBAll(0, 0, 0);
  }

  /**
    * Sets a range of LEDs to one color.
    * @param min
    * @param number
    * @param r - the r value [0-255]
    * @param g - the g value [0-255]
    * @param b - the b value [0-255]
    */
  public void setRangeRGB(int min, int number, int r, int g, int b) {
    if (number != 1) {
      int max = min + number;
      for (int i = min; i < (max); i++) {
        m_ledBuffer.setRGB(i, r, g, b);
      }
    } else {
      m_ledBuffer.setRGB(min, r, g, b);
    }
  }

  // first quadrant is 0, second is 1, third is 2, etc...
  public void setQuadRGB(int quad, int r, int g, int b){
    setRangeRGB((quad * eyesQuarterLength), eyesQuarterLength, r, g, b);
  }

  public void setSlotRGB(int slot, int r, int g, int b){
    setRangeRGB(eyesLength + (slot * stripSlotLength), stripSlotLength, r, g, b);
  }

  /**
   * Sets the battery bling to:
   * Green when battery voltage is greater than 12
   * Blue when battery voltage is greater than 10
   * Red when battery voltage is less than or equal to 10
   * @param quadNum */
  public void setBatteryBling() {
    double volts = RobotController.getBatteryVoltage();

    if (volts > 12) {
      setQuadRGB(1, 0, 255, 0);
      setQuadRGB(2, 0, 255, 0);
    }
    else if (volts > 10){
      setQuadRGB(1, 0, 0, 255);
      setQuadRGB(2, 0, 0, 255);
    }
    else{
      setQuadRGB(1, 255, 0, 0);
      setQuadRGB(2, 255, 0, 0);
    }
  }

  /**
   * Sets the collector bling to orange.
   */
  public void setCollectedBling() {
    setQuadRGB(4, 0, 0, 0);
    setQuadRGB(5, 0, 0, 0);
    setQuadRGB(6, 85, 55, 0);
    setQuadRGB(7, 85, 55, 0);
    setSlotRGB(0, 0, 0, 0);
    setSlotRGB(1, 85, 55, 0);
  }

  /**
   * Sets the feeder bling to orange.
   */
  public void setFeededBling() {
    setQuadRGB(4, 85, 55, 0);
    setQuadRGB(5, 85, 55, 0);
    setQuadRGB(6, 85, 55, 0);
    setQuadRGB(7, 85, 55, 0);
    setSlotRGB(0, 85, 55, 0);
    setSlotRGB(1, 85, 55, 0);
  }

  /**
   * Clears the note bling ring.
   */
  public void setNoNoteBling(){
    setQuadRGB(4, 2, 0, 0);
    setQuadRGB(5, 2, 0, 0);
    setQuadRGB(6, 2, 0, 0);
    setQuadRGB(7, 2, 0, 0);
    setSlotRGB(0, 2, 0, 0);
    setSlotRGB(1, 2, 0, 0);
  }

  /**
   * Sets the note bling ring to green.
   */
  public void setShooterBling(){
    setQuadRGB(4, 0, 255, 0);
    setQuadRGB(5, 0, 255, 0);
    setQuadRGB(6, 0, 255, 0);
    setQuadRGB(7, 0, 255, 0);
  }

  /**
   * TODO:// get vision people to fill this out???
   * Sets the aligned bling to:
   * Green when Note is aligned?
   * Red when Note isn't aligned?
   * @param quadNum */
  public void setAlignedBling() {
    //  setQuadRGB(quadNum1, 255, 0, 0);
    //  setQuadRGB(quadNum2, 255, 0, 0);
     setQuadRGB(0, 0, 255, 0);
     setQuadRGB(3, 0, 255, 0);
  }

  public void setUnalignedBling() {
     setQuadRGB(0, 0, 0, 0);
     setQuadRGB(3, 0, 0, 0);
  }

  public void setRainbowBling(){
    int m_rainbowFirstPixelHue1 = 0;
    int m_rainbowFirstPixelHue2 = 0;
    int count2 = 0;

    for (var i = eyesLength + 1; i < eyesLength + 1 + stripSlotLength; i++) {
      final var hue1 = (m_rainbowFirstPixelHue1 + (i * 180 / stripSlotLength)) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue1, 255, 128);
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue1 += 3;
      // Check bounds
      m_rainbowFirstPixelHue1 %= 180;
    }

    for (var j = eyesLength + 1 + stripSlotLength; j < m_ledBuffer.getLength(); j++) {
      final var hue2 = (m_rainbowFirstPixelHue2 + (count2 * 180 / stripSlotLength)) % 180;
      m_ledBuffer.setHSV(j, hue2, 255, 128);
      count2++;
    }
    m_rainbowFirstPixelHue2 += 3;
    m_rainbowFirstPixelHue2 %= 180;
  }

  // Initialize preferences for this class:
  public static void initPreferences() {
    //TODO: remove if not using preferences
  }
}
