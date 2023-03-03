package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenixpro.signals.InvertedValue;

public final class SwerveConstants {
    public static final boolean OPTIMIZESTEERING = true;
    public static final boolean BOT_IS_NOT_CHARACTERIZED = false;
    public static final boolean RUN_TRAJECTORY = true;

    // SWERVE MODULE CHARACTERISTICS
    // OG WHEEL_RADIUS_METERS = 0.10033/2
    // NEW WHEEL_RADIUS_METERS = 0.13000/2
    public static final double WHEEL_RADIUS_METERS = .10033 / 2;
    public static final double WHEEL_BASE_METERS = 20.1 * 2.54 / 100; // 18 inch wheel base to meters track width is
                                                                        // 24in and wheel base is 22.5 in
    public static final double MAX_SPEED_TICKSper100MS = 21900;
    public static final double STEERING_MOTOR_GEARING = 150.0/7.0; //L3, probably?
    public static final double DRIVE_MOTOR_GEARING = 300.0/49.0; // 1/(14/50*25/19*15/45)
    public static final double SPEED_GOVERNOR = 1; // .11 is a good safe start. Unlock it to "1" when you're confident
                                                   // with the robot
    public static final double TRACK_WIDE = 20.1 * 2.54 / 100;

    // SWERVE Drive Default Values
    public static final double ROBOTHoldAngleKP = 15; // Start at .7 and see where you go from there
    public static final boolean DEFAULT_HOLD_ROBOT_ANGLE = false;
    public static final boolean DEFAULT_FIELD_RELATIVE_DRIVE = true;
    public static final double DEFAULT_HOLD_ROBOT_ANGLE_SETPOINT = 0;
    public static final double kPRotationController = 6.0;
    public static final double kDRotationController = 0;
    public static final double kIRotationController = 0;

    // Swerve Drive Motor IDs
    public static final int FRDriveID = 0;
    public static final int FLDriveID = 10;
    public static final int BRDriveID = 1;
    public static final int BLDriveID = 15;

    // Swerve Steer Motor IDs
    public static final int FRSteerID = 3;
    public static final int FLSteerID = 12;
    public static final int BRSteerID = 2;
    public static final int BLSteerID = 16;

    // Swerve CANCoder Sensor IDs
    public static final int FRSensorID = 2;
    public static final int FLSensorID = 6;
    public static final int BRSensorID = 4;
    public static final int BLSensorID = 8;

    // Swerve CANCoder Sensort offsets
    // CHANGE TO 0 first, reset the sensor,
    // PHYSICALLY zero out the motor
    // place the OPPOSITE of the value
    public static double FRSensorOffset = -0.145752; // 121.904; test? values
    public static double FLSensorOffset = -0.390381;// 40.869;
    public static double BRSensorOffset = -0.376953;// 179.561;
    public static double BLSensorOffset = -0.553223;// -24.873;

    // Give a positive input on the joystick or phoenix tuner
    // Switch this if it goes opposite the desired direction
    // Because of the gearing the convention could be reversed (GUESS AND CHECK)
    public static InvertedValue FRInvertType = InvertedValue.Clockwise_Positive; //Clockwise_Positive
    public static InvertedValue FLInvertType = InvertedValue.CounterClockwise_Positive; //CounterClockwise_Positive
    public static InvertedValue BRInvertType = InvertedValue.Clockwise_Positive; //Clockwise_Positive
    public static InvertedValue BLInvertType = InvertedValue.CounterClockwise_Positive; //CounterClockwise_Positive

    // Swerve Steering PIDs (kP, kI, kD)
    public static Gains FRSteerGains = new Gains(24.8, 0, 0);
    public static Gains FLSteerGains = new Gains(24.8, 0, 0);
    public static Gains BRSteerGains = new Gains(24.8, 0, 0);
    public static Gains BLSteerGains = new Gains(24.8, 0, 0);

    // Swerve Driving PIDs (kP, kI, kD)
    // Once characterized the drive PIDs are meaningless
    public static Gains FRDriveGains = new Gains(0.0007, 0, 0, 1023.0 / 20660.0);
    public static Gains FLDriveGains = new Gains(0.0007, 0, 0, 1023.0 / 20660.0);
    public static Gains BRDriveGains = new Gains(0.0007, 0, 0, 1023.0 / 20660.0);
    public static Gains BLDriveGains = new Gains(0.0007, 0, 0, 1023.0 / 20660.0);

    public static final double kS = 0.1882;
    public static final double kV = 0.69334;
    public static final double kA = 0.10209;
    public static final double kP = 0.00062975; //now better!

    // CTRE CAN-based constants (shouldn't need to change these)
    public static final int kDefaultPIDSlotID = 0;
    public static final int kDefaultClosedLoopError = 1; // degrees
    public static final int kDefaultTimeout = 30;

    // Constants for conversion maths (RARELY THESE SHOULD BE CHANGED)
    public static final double SECONDSper100MS = .1;
    public static final double TICKSperTALONFX_Rotation = 2048;
    public static final double DRIVE_MOTOR_TICKSperREVOLUTION = DRIVE_MOTOR_GEARING * TICKSperTALONFX_Rotation;
    public static final double METERSperWHEEL_REVOLUTION = 2 * Math.PI * WHEEL_RADIUS_METERS;
    public static final double METERSperROBOT_REVOLUTION = 2 * Math.PI
            * Math.hypot(TRACK_WIDE, WHEEL_BASE_METERS);
    public static final double MAX_SPEED_METERSperSECOND = MAX_SPEED_TICKSper100MS / SECONDSper100MS
            / DRIVE_MOTOR_TICKSperREVOLUTION * METERSperWHEEL_REVOLUTION;
    public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND / METERSperROBOT_REVOLUTION
            * (2 * Math.PI);
    public static final double TICKSperTALONFX_STEERING_DEGREE = TICKSperTALONFX_Rotation * STEERING_MOTOR_GEARING
            / 360.0;

    public static class Gains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final double kIzone;
        public final double kPeakOutput;

        /**
         * @param _kP
         * @param _kI
         * @param _kD
         */
        public Gains(double _kP, double _kI, double _kD) {
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = 0;
            kIzone = 0;
            kPeakOutput = 1;
        }

        public Gains(double _kP, double _kI, double _kD, double _kF) {
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            kIzone = 300;
            kPeakOutput = 1;
        }
    }
}
