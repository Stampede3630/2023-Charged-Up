// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.chrono.ThaiBuddhistChronology;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.counter.UpDownCounter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class LEDs extends SubsystemBase implements Loggable{

  public AddressableLED m_led;
  public AddressableLEDBuffer m_LEDBuffer; //156
  public int m_rainbowFirstPixelHue = 0;
  public int r = 0;
  public int g = 0;
  public int b = 0;
  public boolean rainbow = true;
  
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
    if (rainbow)
      beWhoYouAre();
    else
      setEntireStrip();
    // This method will be called once per scheduler run
  }

  public void setRGB(int r, int g, int b) {
    this.r = r;
    this.g = g;
    this.b = b;
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
  
  }

  public void beYellow () {
    setRGB(255, 243, 0);

  }

  public void beIndecisive () {
   setRGB(0, 255, 251);
  }

  public void beWhoYouAre () {
    // For every pixel
    for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      int hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDBuffer.getLength())) % 180;
      // Set the value
      m_LEDBuffer.setHSV(i, hue, 255, 50);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_LEDBuffer);
    m_led.start();
  }

  @Config
  public void setR(int r) {
    this.r = r;
  }

  @Config
  public void setG(int g) {
    this.g = g;
  }

  @Config
  public void setB(int b) {
    this.b = b;
  }

  // @Config
  // public void doRainbowConfig(boolean input) {
  //   this.rainbow = input;
  // }

}
