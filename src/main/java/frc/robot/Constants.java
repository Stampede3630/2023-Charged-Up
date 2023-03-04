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
  public static final class DriverConstants{
    public static final double DRIVE_GOVERNOR = 1;
    public static final boolean FIELD_RELATIVE = true;
    public static final boolean ACCELERATED_INPUTS = false;
  }

  public static final class CannonConstants{
    public static final int SPARK_MASTER_ID = 4;
    public static final int SPARK_FOLLOWER_ID = 11;

    public static final double KP = 0.0013623; //used to be 1/30.0
    public static final double KI = 0;
    public static final double KD = 0.000517; //used to be 0
    public static final double KS = 0.00;
    public static final double KG = 0.73;
    public static final double KV = 2.44;
    public static final double KA = 0.06;
    public static final double KAM = 7.71E-3;
    public static final double KGM = 0.0289;
    public static final double KAB = -0.133;
    public static final double KGB = 8.57E-3;

    public static final double CONVERSION_FACTOR = 360;
    public static final double ZERO_OFFSET = 294.0;
    public static final int  CURRENT_LIMIT = 50; //used to be 70
    public static final float FORWARD_LIMIT = 195.0f;
    public static final float REVERSE_LIMIT = -15.0f;
    public static final double INITIALIZED_ANGLE = 30.0;
    public static final double ERROR = 10.0;
  }

  public static final class ExtendoConstants{
    public static final int SPARK_MAX_ID = 17;
    public static final double INITIALIZED_INCHES = 1.0;
    public static final int CURRENT_LIMIT = 40; //changed from 50
    public static final double MAGIC_TO_INCHES = 1.802406002431152;
    public static final double ERROR = 5.0;
    
    public static final double KP = 1.0/6.0; //used to be 1/30.0
    public static final double KI = 0;
    public static final double KD = 0; //used to be 0

  }

  public static final class IntakeConstants{
    public static final int SPARK_MAX_ID = 14;
    public static final int CURRENT_LIMIT = 40;
    public static final double GAME_PIECE_DETECTION_AMPS = 55;
  }

  public static final class LidConstants{
    public static final int SPARK_MAX_ID = 19;
    public static final double INITIALIZED_ANGLE = 180;
    public static final double CONVERSION_FACTOR = 360;
    public static final float FORWARD_LIMIT = 310.0f;
    public static final float REVERSE_LIMIT = 90.0f;
    public static final double ZERO_OFFSET = 360-87;
    public static final int  CURRENT_LIMIT = 20; 

    public static final double KP = 1.0/10.0; //used to be 1/30.0
    public static final double KI = 0.0;
    public static final double KD = 0.0; //used to be 0
  }


  public static final Translation2d blueScorePos[] = new Translation2d[] {
    new Translation2d(0, 0),
    new Translation2d(0, 0),
    new Translation2d(0, 0)
  };

}
