package frc.robot.subsystems.swerve;



import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.sim.TalonFXSimState;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;


public class SwerveModuleSim {

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER *Math.PI;
    // Simulation

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerSensorCANCoder;
    public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward( // real
    SwerveConstants.kS, // Voltage to break static friction
    SwerveConstants.kV, // Volts per meter per second
    SwerveConstants.kA // Volts per meter per second squared
    );
    // Steer feed forward
    public static final SimpleMotorFeedforward STEER_FF = new SimpleMotorFeedforward( // real
        0.55, // Voltage to break static friction
        0.23, // Volts per radian per second
        0.0056 // Volts per radian per second squared
    );
    private final FlywheelSim driveWheelSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            DRIVE_FF.kv * WHEEL_CIRCUMFERENCE / (2*Math.PI),
            DRIVE_FF.ka * WHEEL_CIRCUMFERENCE / (2*Math.PI)
        ),
        DCMotor.getFalcon500(1),
        SwerveConstants.DRIVE_MOTOR_GEARING
    );

    private final FlywheelSim steeringSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(STEER_FF.kv, STEER_FF.ka),
        DCMotor.getFalcon500(1),
        SwerveConstants.STEERING_MOTOR_GEARING
    );
    public SwerveModuleSim(TalonFX driveMotor, TalonFX steerMotor, CANcoder steerSensorCANCoder) {
        this.driveMotor= driveMotor;
        this.steerMotor= steerMotor;
        this.steerSensorCANCoder = steerSensorCANCoder;


    }

    public void simulationPeriodic(double deltaTime){
        TalonFXSimState driveSimState = driveMotor.getSimState();
        TalonFXSimState steerSimState = steerMotor.getSimState();
        // apply our commanded voltage to our simulated physics mechanisms
        double driveVoltage = driveSimState.getMotorVoltage();
        if(driveVoltage >= 0) driveVoltage = Math.max(0, driveVoltage- STEER_FF.ks);
        else driveVoltage = Math.min(0, driveVoltage+ STEER_FF.ks);
        driveSimState.setSupplyVoltage(driveVoltage);

        double steerVoltage = steerSimState.getMotorVoltage();
        if(steerVoltage >= 0) steerVoltage = Math.max(0, steerVoltage- STEER_FF.ks);
        else steerVoltage = Math.min(0, steerVoltage+ STEER_FF.ks);
        steerSimState.setSupplyVoltage(steerVoltage);
        
        driveWheelSim.update(deltaTime);
        steeringSim.update(deltaTime);

        // update our simulated devices with our simulated physics results
        double driveMotorVelocityNative = rotationsToVelocity(driveWheelSim.getAngularVelocityRPM()/60, SwerveConstants.DRIVE_MOTOR_GEARING);
        double driveMotorPositionDeltaNative = driveMotorVelocityNative*10*deltaTime;
        driveSimState.setRotorVelocity(driveMotorVelocityNative);
        driveSimState.addRotorPosition(driveMotorPositionDeltaNative);
        //driveSimState..setSupplyCurrent(driveWheelSim.getCurrentDrawAmps()/2);

        //SmartDashboard.putNumber("Steer Sim Model Velocity", steeringSim.getAngularVelocityRPM());
        double steerMotorVelocityNative = rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, SwerveConstants.STEERING_MOTOR_GEARING);
        double steerMotorPositionDeltaNative = steerMotorVelocityNative*10*deltaTime;
        steerSimState.setRotorVelocity(steerMotorVelocityNative);
        steerSimState.addRotorPosition(steerMotorPositionDeltaNative);
        //System.out.println(steerMotorPositionDeltaNative);
        //steerMotorSim.setSupplyCurrent(steeringSim.getCurrentDrawAmps()/2);
        steerSensorCANCoder.getSimState().setVelocity(steerMotorVelocityNative);
        steerSensorCANCoder.getSimState().addPosition(steerMotorPositionDeltaNative);
        //steerSensorCANCoderSim.setVelocity((int)(rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, 1)*2));
        //steerSensorCANCoderSim.setRawPosition((int)(getIntegratedHeading().getDegrees()/360.0*4096));

        //driveMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        //steerMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        //steerEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());
    }

    // public double getDriveCurrentDraw(){
    //     return driveMotor.getSupplyCurrent();
    // }
    // public double getSteerCurrentDraw(){
    //     return steerMotor.getSupplyCurrent();
    // }

    public static double positionToRotations(double nativePosition, double motorRotationsPerMechanismRotation){
        return nativePosition / 2048 / motorRotationsPerMechanismRotation;
    }
    public static double positionToDegrees(double nativePosition, double motorRotationsPerMechanismRotation){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * 360;
    }
    public static double positionToRadians(double nativePosition, double motorRotationsPerMechanismRotation){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * 2 * Math.PI;
    }
    public static double positionToMeters(double nativePosition, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * circumferenceMeters;
    }
    public static double rotationsToPosition(double rotations, double motorRotationsPerMechanismRotation){
        return rotations * 2048 * motorRotationsPerMechanismRotation;
    }
    public static double degreesToPosition(double degrees, double motorRotationsPerMechanismRotation){
        return rotationsToPosition(degrees/(360), motorRotationsPerMechanismRotation);
    }
    public static double radiansToPosition(double radians, double motorRotationsPerMechanismRotation){
        return rotationsToPosition(radians/(2*Math.PI), motorRotationsPerMechanismRotation);
    }
    public static double metersToPosition(double meters, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return rotationsToPosition(meters/(circumferenceMeters), motorRotationsPerMechanismRotation);
    }

    public static double velocityToRotations(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return nativeVelocity * 10 / 2048 / motorRotationsPerMechanismRotation;
    }
    public static double velocityToDegrees(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * 360;
    }
    public static double velocityToRadians(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * 2 * Math.PI;
    }
    public static double velocityToMeters(double nativeVelocity, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * circumferenceMeters;
    }
    public static double rotationsToVelocity(double rotations, double motorRotationsPerMechanismRotation){
        return rotations * 2048 / 10 * motorRotationsPerMechanismRotation;
    }
    public static double degreesToVelocity(double degrees, double motorRotationsPerMechanismRotation){
        return rotationsToVelocity(degrees/(360), motorRotationsPerMechanismRotation);
    }
    public static double radiansToVelocity(double radians, double motorRotationsPerMechanismRotation){
        return rotationsToVelocity(radians/(2*Math.PI), motorRotationsPerMechanismRotation);
    }
    public static double metersToVelocity(double meters, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return rotationsToVelocity(meters/(circumferenceMeters), motorRotationsPerMechanismRotation);
    }


}
