package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable{
    private CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.SPARK_MAX_ID, MotorType.kBrushless);
    private RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();
    private SparkMaxPIDController m_intakePid = m_intakeMotor.getPIDController();
    private SparkMaxLimitSwitch intakeHardStop = m_intakeMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    private double speed = 0;
    private boolean haveGamePiece = false;
    public Intake() {

        m_intakeMotor.setSmartCurrentLimit(70);


                
        // cannonExtension.setInverted(true);
        //changed idle mode to help with troubleshooting    
        m_intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeHardStop.enableLimitSwitch(true);

        // m_intakePid.setFeedbackDevice(m_intakeEncoder);
        // m_intakePid.setPositionPIDWrappingEnabled(false);

        // m_intakePid.setP(Preferences.getDouble("IntakeKP", 1.0/30.0));
        // m_intakePid.setI(Preferences.getDouble("IntakeKI", 0.0));
        // m_intakePid.setD(Preferences.getDouble("IntakeKD", 0.0));
        // m_intakePid.setOutputRange(-.5, .5);

        m_intakeMotor.burnFlash();
        

    }

    @Override
    public void periodic() {
        m_intakeMotor.set(speed);
    }

    public void setIntake(double intakeSpeed) {
        speed = intakeSpeed;
    }

    public void stopIntake() {
        speed = 0;
    }

    public boolean getIntakeHardStop(){
      return intakeHardStop.isPressed();
    }

    // @Log.Graph
    public double getIntakeCurrent() {
        return m_intakeMotor.getOutputCurrent();
    }

    @Log
    public boolean haveGamePiece() {
        return haveGamePiece;
    }

    public void leaveGamePiece(){
        haveGamePiece = false;
    }

    public boolean checkForCurrentSpike() {
        haveGamePiece = (getIntakeCurrent() > IntakeConstants.GAME_PIECE_DETECTION_AMPS) || (intakeHardStop.isPressed()) ? true : haveGamePiece;
        return haveGamePiece;
    }
  @Config
  public void setHaveGamePiece(boolean input) {
      this.haveGamePiece = input;
  }
}
