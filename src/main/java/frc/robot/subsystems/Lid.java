package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LidConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Lid extends SubsystemBase implements Loggable{
    private CANSparkMax m_lidMotor = new CANSparkMax(LidConstants.SPARK_MAX_ID, MotorType.kBrushless);
    private SparkMaxAbsoluteEncoder m_lidAbsolute = m_lidMotor.getAbsoluteEncoder(Type.kDutyCycle);
    private SparkMaxPIDController m_lidPid = m_lidMotor.getPIDController();
    public double lidReference = LidConstants.INITIALIZED_ANGLE;

    public Lid() {
        m_lidAbsolute.setInverted(true);
        m_lidAbsolute.setPositionConversionFactor(LidConstants.CONVERSION_FACTOR);
        m_lidAbsolute.setVelocityConversionFactor(LidConstants.CONVERSION_FACTOR);
        m_lidMotor.setSoftLimit(SoftLimitDirection.kForward, LidConstants.FORWARD_LIMIT);
        m_lidMotor.setSoftLimit(SoftLimitDirection.kReverse, LidConstants.REVERSE_LIMIT);
        m_lidAbsolute.setZeroOffset(LidConstants.ZERO_OFFSET);

        m_lidMotor.setSmartCurrentLimit(LidConstants.CURRENT_LIMIT);
                
        // cannonExtension.setInverted(true);
        //changed idle mode to help with troubleshooting    
        m_lidMotor.setIdleMode(IdleMode.kBrake);

        m_lidPid.setFeedbackDevice(m_lidAbsolute);
        m_lidPid.setPositionPIDWrappingEnabled(false);

        m_lidPid.setP(Preferences.getDouble("LidKP", LidConstants.KP));
        m_lidPid.setI(Preferences.getDouble("LidKI", LidConstants.KI));
        m_lidPid.setD(Preferences.getDouble("LidKD", LidConstants.KD));
        m_lidPid.setOutputRange(-1 , 1);

        m_lidMotor.burnFlash();
    }

    @Override
    public void periodic() {
        m_lidPid.setReference(lidReference, ControlType.kPosition);

        if (Preferences.getBoolean("Wanna PID Lid", false)) {
            m_lidPid.setP(Preferences.getDouble("LidKP", LidConstants.KP));
            m_lidPid.setI(Preferences.getDouble("LidKI", LidConstants.KI));
            m_lidPid.setD(Preferences.getDouble("LidKD", LidConstants.KD));
    
            Preferences.setBoolean("Wanna PID Lid", false);
        }
    }

    public void setLid(double intakeLidAngle){
        setLidReference(intakeLidAngle);
    }

    @Config(defaultValueNumeric = 180)
    public void setLidReference(double input) {
        this.lidReference = input;
    }

    @Log
    public double lidPosition() {
        return m_lidAbsolute.getPosition();
    }
    
    @Log
    public double getLidCurrent() {
        return m_lidMotor.getOutputCurrent();
    }
}
