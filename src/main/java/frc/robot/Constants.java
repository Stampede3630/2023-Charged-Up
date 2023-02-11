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


public enum poiCannon{
  high(0, 0),
  mid(1, 1),
  low(2, 2),

  doubleSubstation(3, 3),
  singleSubstation(4, 4),
  groundPickup(5, 5);

  private double cannonAngle;
  private double extension;

  private poiCannon(double cannonAngle, double extension){
    this.cannonAngle = cannonAngle;
    this.extension = extension;
  }

  public double getCannonAngle(){
    return cannonAngle;
  }

  public double getExtension(){
    return extension;
  }
}

public enum poiDrive{
  humanPlayerCubeNode(1.7, 4.41, 0),
  coopertitionCubeNode(1.7, 2.73, 0),
  wallCubeNode(1.7, 1.07, 0),

  substationSingle(14.01, 7.62, 90),
  substationDouble(15.89, 6.65, 0);

  private double x;
  private double y;
  private double rot;

  private poiDrive(double x, double y, double rot){
    this.x = x;
    this.y = y;
    this.rot = rot;
  }
  public double getX(){
    return x;
  }

  public double getY(){
    return y;
  }

  public double getRot(){
    return rot;
  }
}

}
