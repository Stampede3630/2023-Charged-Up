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
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Lid extends SubsystemBase implements Loggable{
    private CANSparkMax m_lidMotor = new CANSparkMax(19, MotorType.kBrushless);
    private SparkMaxAbsoluteEncoder m_lidAbsolute = m_lidMotor.getAbsoluteEncoder(Type.kDutyCycle);
    private SparkMaxPIDController m_lidPid = m_lidMotor.getPIDController();
    public double lidReference = 180;

    public Lid() {
        m_lidAbsolute.setInverted(true);
        m_lidAbsolute.setPositionConversionFactor(360.0);
        m_lidAbsolute.setVelocityConversionFactor(360.0);
        m_lidMotor.setSoftLimit(SoftLimitDirection.kForward, 310);
        m_lidMotor.setSoftLimit(SoftLimitDirection.kReverse, 90);
        m_lidAbsolute.setZeroOffset(360-87);

        m_lidMotor.setSmartCurrentLimit(20);
                
        // cannonExtension.setInverted(true);
        //changed idle mode to help with troubleshooting    
        m_lidMotor.setIdleMode(IdleMode.kBrake);

        m_lidPid.setFeedbackDevice(m_lidAbsolute);
        m_lidPid.setPositionPIDWrappingEnabled(false);

        m_lidPid.setP(Preferences.getDouble("LidKP", 1.0/30.0));
        m_lidPid.setI(Preferences.getDouble("LidKI", 0.0));
        m_lidPid.setD(Preferences.getDouble("LidKD", 0.0));
        m_lidPid.setOutputRange(-1 , 1);

        m_lidMotor.burnFlash();
    }

    @Override
    public void periodic() {
        m_lidPid.setReference(lidReference, ControlType.kPosition);

        if (Preferences.getBoolean("Wanna PID Lid", false)) {
            m_lidPid.setP(Preferences.getDouble("LidKP", 1.0/10.0));
            m_lidPid.setI(Preferences.getDouble("LidKI", 0.0));
            m_lidPid.setD(Preferences.getDouble("LidKD", 0.0));
    
            Preferences.setBoolean("Wanna PID Lid", false);
        }
    }

    public void setLipIn() {
        lidReference = 70;
    }

    public void setLipOut() {
        lidReference = 180;
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
