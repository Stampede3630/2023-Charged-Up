// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  public AddressableLED m_led;
  public AddressableLEDBuffer m_LEDBuffer; //156
  public int m_rainbowFirstPixelHue = 0;
  public int r = 0;
  public int g = 0;
  public int b = 0;
  private final int HUE_CHANGE_PER_SEC = 60;
  private final double CHASE_MOVE_PER_MS = 0.0333;

  private final double STROBE_TIME_MS = 500;
  private double previousTime = System.currentTimeMillis();
  private final Color[] strobeColors = {new Color(0,0,255),new Color(255,255,0)};
  private double chaseSeparator = 0; 
  private LEDMode mode = LEDMode.RAINBOW;
  
  /** Creates a new LEDs. */
  public LEDs() {
    this(9, 156);
  }

  public LEDs(int port, int length) {
    m_led = new AddressableLED(9);
    m_led.setLength(length);
    m_LEDBuffer = new AddressableLEDBuffer(length);
  }

  @Override
  public void periodic() {
     switch (mode) {
       case STROBE: break;
       case CHASING: chaseColorsTwo(); break;
       case SOLID: setEntireStrip(); break;
       case RAINBOW: beWhoYouAre(); break;
       default: off();
     }
    // This method will be called once per scheduler run
  }

  public void setMode(LEDMode mode) {
    this.mode = mode;
  }

  public String getMode() {
      return mode.name();
  }

  public void setRGB(int r, int g, int b) {
    this.r = r;
    this.g = g;
    this.b = b;
  }

  public void off() {
    setRGB(0,0,0);
  }

  private void setEntireStrip() {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_LEDBuffer.setRGB(i, r, g, b);
   }
   m_led.setData(m_LEDBuffer);
   m_led.start();
  }
  

  public void bePurple () {
    setRGB(162, 0, 255);
    setMode(LEDMode.SOLID);
  }

  public void beYellow () {
    setRGB(255, 243, 0);
    setMode(LEDMode.SOLID);
  }
  public void chaseColors() {

    for (int i = 0; i < chaseSeparator; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // Set the value to claw machine + bing chilling 

      m_LEDBuffer.setLED(i, strobeColors[0]);
    }
    for (int i = 0; i < m_LEDBuffer.getLength()-chaseSeparator; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // Set the value to claw machine + bing chilling 

      m_LEDBuffer.setLED(i, strobeColors[1]);
       
    }
    chaseSeparator++;
    chaseSeparator %= m_LEDBuffer.getLength();
    m_led.setData(m_LEDBuffer);
    m_led.start();
  }

  public void chaseColorsTwo() {
    double currentTime = System.currentTimeMillis();
    double timeElapsed = currentTime-previousTime;
    int length = m_LEDBuffer.getLength();
    int delim1 = (int) chaseSeparator;
    int delim2 = (int) (length/2.0+chaseSeparator);
     for (int i = delim1; i < delim2+(delim1>delim2 ? length : 0); i++) { // wrap if 1 is further than 2
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // Set the value to claw machine + bing chilling 

      m_LEDBuffer.setLED(i%length, strobeColors[0]);
    }
    for (int i = delim2; i < delim1+(delim1>delim2 ? 0 : length); i++) { // wrap if 2 is further than 1
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // Set the value to claw machine + bing chilling 

      m_LEDBuffer.setLED(i%length, strobeColors[1]);
       
    }
    chaseSeparator += CHASE_MOVE_PER_MS*timeElapsed;
    chaseSeparator %= m_LEDBuffer.getLength();
    m_led.setData(m_LEDBuffer);
    m_led.start();
    previousTime = currentTime;
  }
  public void beWhoYouAre () {
    double currentTime = System.currentTimeMillis();
    double timeElapsed = currentTime-previousTime;
    // For every pixel
    for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      int hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDBuffer.getLength())) % 180;
      // Set the value
      m_LEDBuffer.setHSV(i, hue, 255, 50);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += HUE_CHANGE_PER_SEC*(timeElapsed/1000.0);
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_LEDBuffer);
    m_led.start();
    previousTime = currentTime;
  }

  public void setR(int r) {
    this.r = r;
  }

  public void setG(int g) {
    this.g = g;
  }

  public void setB(int b) {
    this.b = b;
  }

  public enum LEDMode {
    STROBE, CHASING, RAINBOW, SOLID, OFF;
  }

  // @Config
  // public void doRainbowConfig(boolean input) {
  //   this.rainbow = input;
  // }
}
