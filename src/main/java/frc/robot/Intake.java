package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable{
    private CANSparkMax m_intakeMotor = new CANSparkMax(14, MotorType.kBrushless);
    private RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();
    private SparkMaxPIDController m_intakePid = m_intakeMotor.getPIDController();

    private double speed = 0;
    private boolean haveGamePiece = false;
    public Intake() {

        m_intakeMotor.setSmartCurrentLimit(70);
                
        // cannonExtension.setInverted(true);
        //changed idle mode to help with troubleshooting    
        m_intakeMotor.setIdleMode(IdleMode.kBrake);

        // m_intakePid.setFeedbackDevice(m_intakeEncoder);
        // m_intakePid.setPositionPIDWrappingEnabled(false);

        // m_intakePid.setP(Preferences.getDouble("IntakeKP", 1.0/30.0));
        // m_intakePid.setI(Preferences.getDouble("IntakeKI", 0.0));
        // m_intakePid.setD(Preferences.getDouble("IntakeKD", 0.0));
        // m_intakePid.setOutputRange(-.5, .5);

        // m_lidMotor.burnFlash();
    }

    @Override
    public void periodic() {
        m_intakeMotor.set(speed);
    }

    public void runIntake() {
        speed = 0.75;
    }

    public void reverseIntake() {
        speed = -0.75;
    }

    public void stopIntake() {
        speed = 0;
    }

    @Config
    public void setSpeed(double input) {
        this.speed = input;
    }

    @Log
    public double getIntakeCurrent() {
        return m_intakeMotor.getOutputCurrent();
    }

    @Log
    public boolean haveGamePiece() {
        return haveGamePiece;
    }
}
