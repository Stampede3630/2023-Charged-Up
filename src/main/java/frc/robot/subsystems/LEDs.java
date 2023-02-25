// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePieceOrientation;
import frc.robot.RobotContainer.GamePieceType;

public class LEDs extends SubsystemBase {

  public AddressableLED m_led = new AddressableLED(0);
  public AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(60);
  public int m_rainbowFirstPixelHue = 0;
  private int r = 0;
  public int g = 0;
  public int b = 0;
  
  /** Creates a new LEDs. */
  public LEDs() {

  }

  @Override
  public void periodic() {

    // if (GamePieceType.CONE != null){
    //   becomeYellow();
    // } else if(GamePieceType.CUBE != null){
    //   becomePurple();
    // } else{
    //   becomeRainbow();
    // }
    // This method will be called once per scheduler run
  }

  public void setEntireStrip(int r, int g, int b) {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_LEDBuffer.setRGB(i, r, g, r);
   }
   
   m_led.setData(m_LEDBuffer);
  }

  public void bePurple () {
    setEntireStrip(162, 0, 255);
    // setEntireStrip(r, g, b);
  }

  public void beYellow () {
    setEntireStrip(255, 243, 0);
    // setEntireStrip(r, g, b);
  }



  public void beIndecisive () {
   setEntireStrip(0, 255, 251);
  }

  public void beWhoYouAre () {
    // For every pixel
    for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDBuffer.getLength())) % 180;
      // Set the value
      m_LEDBuffer.setHSV(i, hue, 100, 100);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

}
