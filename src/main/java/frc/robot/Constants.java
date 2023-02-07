// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static double driveGovernor = 1;
  public final static boolean fieldRelative = true;
  public static final boolean acceleratedInputs = false;

  public static final Translation2d blueScorePos[] = new Translation2d[] {
    new Translation2d(0, 0),
    new Translation2d(0, 0),
    new Translation2d(0, 0)
  };


public enum poi{
  highLeft(0, 0, 0, 0),
  highMid(1, 1, 1, 1),
  highRight(2, 2, 2, 2),

  doubleSubstation(3, 3, 3, 3),
  singleSubstation(4, 4, 4, 4),
  groundPickup(5, 5, 5, 5);

  private double cannonAngle;
  private double extension;
  private double xCoord;
  private double yCoord;

  private poi(double cannonAngle, double extension, double xCoord, double yCoord){
    this.cannonAngle = cannonAngle;
    this.extension = extension;
    this.xCoord = xCoord;
    this.yCoord = yCoord;
  }

  public double getCannonAngle(){
    return cannonAngle;
  }

  public double getExtension(){
    return extension;
  }
  public double getXCoord(){
    return xCoord;
  }
  public double getYCoord(){
    return yCoord;
  }

}

}
