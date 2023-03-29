package frc.robot.subsystems.swerve;

import com.ctre.phoenixpro.*;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;

public class SwerveModule {

    private StatusSignalValue<Double> m_drivePosition;
    private StatusSignalValue<Double> m_driveVelocity;
    private StatusSignalValue<Double> m_steerPosition;
    private StatusSignalValue<Double> m_steerVelocity;
    private BaseStatusSignalValue[] m_signals;
    private double m_driveRotationsPerMeter = 0;

    TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
    TalonFXConfiguration steerTalonConfig = new TalonFXConfiguration();
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    private SwerveModulePosition m_internalState = new SwerveModulePosition();
    private PositionDutyCycle m_angleSetter = new PositionDutyCycle(0);
    private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);

    boolean readyToSeed = false;
    boolean readyToCANCoderAbs = false;
    boolean canCoderAbsSuccessful = false;
    boolean prepareForSeeding = true;
    boolean seedingOccured = false;
    boolean seedingSuccessful = false;
    double seedingTimer;
    boolean guudder;
    public final SteeringMotor steeringMotor;
    public final SteeringSensor steeringSensor;
    public final DriveMotor driveMotor;
    public final Translation2d moduleXYTranslation;
    public final SwerveModuleSim simModule;
    public String steerMode = "INTEGRATED";
    public boolean hasSwerveSeedingOccurred = false;
    public boolean hasCANCoderBeenSetToAbs = false;
    public double swerveSeedingRetryCount = 0;
    
    public static SimpleMotorFeedforward driveMotorFeedforward = new SimpleMotorFeedforward(SwerveConstants.kS,
            SwerveConstants.kV, SwerveConstants.kA);

    /**
     * @param moduleXYTranslation X is NorthSouth and Y is EastWest
     * 
     *                            Helpful hints:
     *                            1. when determining your steering motor offsets
     *                            first rotate
     *                            all modules to face a certain direction
     *                            (inward/outward/left/right)
     *                            2. Once that's done make sure you determine which
     *                            drive motors need to go
     *                            clockwise positive/negative
     *                            3. NOW, ur ready to play with the offsets
     *                            4. Use phoenix tuner to determin PID coefficients
     *                            for EACH wheel, each wheel may
     *                            may be slightly to vastly different
     * 
     */
    public SwerveModule(DriveMotor driveMotor, SteeringMotor steeringMotor, SteeringSensor steeringSensor,
            Translation2d moduleXYTranslation) {
        this.steeringMotor = steeringMotor;
        this.steeringSensor = steeringSensor;
        this.driveMotor = driveMotor;
        this.moduleXYTranslation = moduleXYTranslation;


        if (RobotBase.isSimulation()) {
            simModule = new SwerveModuleSim(driveMotor, steeringMotor, steeringSensor);
        } else {
            simModule = null;
        }

        swerveModuleInit();
    }

    private void swerveModuleInit() {
        driveMotor.getConfigurator().refresh(driveTalonConfig);
        final Slot0Configs DriveMotorGains = new Slot0Configs();

        DriveMotorGains.kP = SwerveConstants.kP;
        DriveMotorGains.kI = 0;
        DriveMotorGains.kD = 0;
        DriveMotorGains.kS = SwerveConstants.kS;
        DriveMotorGains.kV = SwerveConstants.kV;
        // driveMotor.getConfigurator().apply(DriveMotorGains);

        driveTalonConfig.Slot0 = DriveMotorGains;

        // MotorOutputConfigs driveMotorCOnfigs = new MotorOutputConfigs();
        driveTalonConfig.MotorOutput.Inverted = driveMotor.kWheelDirectionType;
        driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        driveMotor.getConfigurator().apply(driveTalonConfig);

        steeringMotor.getConfigurator().refresh(steerTalonConfig);
        
        final Slot0Configs SteerMotorGains = new Slot0Configs();
        SteerMotorGains.kP = steeringMotor.kGAINS.kP; 
        SteerMotorGains.kI = steeringMotor.kGAINS.kI; 
        SteerMotorGains.kD = steeringMotor.kGAINS.kD; 
        steerTalonConfig.Slot0 = SteerMotorGains;
        // Modify configuration to use remote CANcoder fused
        steerTalonConfig.Feedback.FeedbackRemoteSensorID = steeringSensor.getDeviceID() ;
        steerTalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerTalonConfig.Feedback.RotorToSensorRatio = SwerveConstants.STEERING_MOTOR_GEARING;
        steerTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerTalonConfig.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for swerve modules
        
        steeringMotor.getConfigurator().apply(steerTalonConfig);

        steeringSensor.getConfigurator().refresh(canCoderConfig);

        canCoderConfig.MagnetSensor.MagnetOffset = steeringSensor.kOffsetDegrees;
        steeringSensor.getConfigurator().apply(canCoderConfig);

        // driveMotor.getConfigurator().apply(talonConfigs);

        m_drivePosition = driveMotor.getPosition();
        m_driveVelocity = driveMotor.getVelocity();
        m_steerPosition = steeringSensor.getPosition();
        m_steerVelocity = steeringSensor.getVelocity();

        m_signals = new BaseStatusSignalValue[4];
        m_signals[0] = m_drivePosition;
        m_signals[1] = m_driveVelocity;
        m_signals[2] = m_steerPosition;
        m_signals[3] = m_steerVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = SwerveConstants.DRIVE_MOTOR_GEARING;
        double metersPerWheelRotation = 2 * Math.PI * (SwerveConstants.WHEEL_RADIUS_METERS);
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
  
    }

    public void setModuleToCoast() {
        driveMotor.getConfigurator().refresh(driveTalonConfig);
        steeringMotor.getConfigurator().refresh(steerTalonConfig);

        driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        steerTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveTalonConfig);
        steeringMotor.getConfigurator().apply(steerTalonConfig);
    }

    public void setModuleToBrake() {
        driveMotor.getConfigurator().refresh(driveTalonConfig);
        steeringMotor.getConfigurator().refresh(steerTalonConfig);

        driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveTalonConfig);
        steeringMotor.getConfigurator().apply(steerTalonConfig);
    }

    /**
     * This method takes our best crack at seeding the angle from CANCoder to
     * Integrated Sensor on the Steering Motor (cuz faster/snappier).
     */

    public SwerveModulePosition getPosition() {
    /* Refresh all signals */
    m_drivePosition.refresh();
    m_driveVelocity.refresh();
    m_steerPosition.refresh();
    m_steerVelocity.refresh();

    /* Now latency-compensate our signals */
    double drive_rot =
            m_drivePosition.getValue()
                    + (m_driveVelocity.getValue() * m_drivePosition.getTimestamp().getLatency());
    double angle_rot =
            m_steerPosition.getValue()
                    + (m_steerVelocity.getValue() * m_steerPosition.getTimestamp().getLatency());

    /* And push them into a SwerveModuleState object to return */
    m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
    /* Angle is already in terms of steer rotations */
    m_internalState.angle = Rotation2d.fromRotations(angle_rot);

    return m_internalState;
}

    public String getSteeringSelectedSensor() {
        return steerMode;
    }

    // public void setDesiredState(SwerveModuleState desiredState) {
    //     SwerveModuleState kState = desiredState;
    //     if (Preferences.getBoolean("pOptimizeSteering", SwerveConstants.OPTIMIZESTEERING)) {
    //         kState = optimize(desiredState, new Rotation2d(Math.toRadians(steeringMotor.getSelectedSensorPosition())));
    //     }
    // double convertedspeed = kState.speedMetersPerSecond * (SwerveConstants.SECONDSper100MS)
        //         * SwerveConstants.DRIVE_MOTOR_TICKSperREVOLUTION / (SwerveConstants.METERSperWHEEL_REVOLUTION);
        // setSteeringAngle(kState.angle.getDegrees());

        // if (SwerveConstants.BOT_IS_NOT_CHARACTERIZED) {

        //     driveMotor.set(ControlMode.PercentOutput,
        //             kState.speedMetersPerSecond / SwerveConstants.MAX_SPEED_METERSperSECOND);

        // } else {
        //     // System.out.println(driveMotorFeedforward.calculate(kState.speedMetersPerSecond));
        //     driveMotor.setVoltage(driveMotorFeedforward.calculate(kState.speedMetersPerSecond));
        // }

    public void setDesiredState(SwerveModuleState desiredState) {
        var optimized = SwerveModuleState.optimize(desiredState, m_internalState.angle);

        double angleToSetDeg = optimized.angle.getRotations();
        steeringMotor.setControl(m_angleSetter.withPosition(angleToSetDeg).withSlot(0));
        double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;
        driveMotor.setControl(m_velocitySetter.withVelocity(velocityToSet));

    }

    // public SwerveModuleState getSwerveModuleState() {
    //     return new SwerveModuleState(
    //             driveMotor.getSelectedSensorVelocity() / SwerveConstants.SECONDSper100MS
    //                     / SwerveConstants.DRIVE_MOTOR_TICKSperREVOLUTION * SwerveConstants.METERSperWHEEL_REVOLUTION,
    //             Rotation2d.fromDegrees(steeringMotor.getSelectedSensorPosition()));
    // }

    /*
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */


    /*
     * This method takes in setAngle in DEGREES,
     * 
     * compares that angle with the current position of
     * the swerve module and decides which direction to
     * rotate the module.
     * 
     * The angle is then converted to sensor units (4096
     * equals 1 full rotation) units equals and fed
     * to the steering motor to update.
     * 
     */
    // public void setSteeringAngle(double _angle) {
    //     // double newAngleDemand = _angle;
    //     double currentSensorPosition = steeringMotor.getSelectedSensorPosition();
    //     double remainder = Math.IEEEremainder(currentSensorPosition, 360);
    //     double newAngleDemand = _angle + currentSensorPosition - remainder;

    //     // System.out.println(mSteeringMotor.getSelectedSensorPosition()-remainder );
    //     if (newAngleDemand - currentSensorPosition > 180.1) {
    //         newAngleDemand -= 360;
    //     } else if (newAngleDemand - currentSensorPosition < -180.1) {
    //         newAngleDemand += 360;
    //     }
    //     steeringMotor.set(ControlMode.Position, newAngleDemand);
    // }



    // public SwerveModuleState optimize(
    //         SwerveModuleState desiredState, Rotation2d currentAngle) {
    //     var delta = desiredState.angle.minus(currentAngle);
    //     if (Math.abs(delta.getDegrees()) > 90.0) { // SJV: If this doesn'twork try 360
    //         return new SwerveModuleState(
    //                 -desiredState.speedMetersPerSecond,
    //                 desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    //     } else {
    //         return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    //     }
    // }

    public static class SteeringMotor extends TalonFX {
        public SwerveConstants.Gains kGAINS;

        public SteeringMotor(int _talonID, SwerveConstants.Gains _gains) {
            super(_talonID, "Swerve");
            kGAINS = _gains;
        }
    }

    public static class DriveMotor extends TalonFX {
        public InvertedValue kWheelDirectionType;
        public SwerveConstants.Gains kGAINS;

        public DriveMotor(int _talonID, InvertedValue kWheelDirectionType, SwerveConstants.Gains _gains) {
            super(_talonID, "Swerve");
            
            this.kWheelDirectionType = kWheelDirectionType;
            kGAINS = _gains;
        }
    }

    public static class SteeringSensor extends CANcoder {
        public double kOffsetDegrees;

        public SteeringSensor(int _sensorID, double _offsetDegrees) {
            super(_sensorID, "Swerve");
            kOffsetDegrees = _offsetDegrees;
        }
    }

}
