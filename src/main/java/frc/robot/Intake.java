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

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

        m_intakeMotor.setSmartCurrentLimit(80);

                
        // cannonExtension.setInverted(true);
        //changed idle mode to help with troubleshooting    
        m_intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeHardStop.enableLimitSwitch(true); 

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

    @Log.Graph
    public double getIntakeCurrent() {
        return m_intakeMotor.getOutputCurrent();
    }

    @Log.BooleanBox(tabName = "nodeSelector")
    public boolean haveGamePiece() {
        return haveGamePiece;
    }

    public void leaveGamePiece(){
        haveGamePiece = false;
    }
    

    public boolean checkForGamePiece() {
        haveGamePiece = intakeHardStop.isPressed() ? true : haveGamePiece; // latching
        return haveGamePiece;
    }
  @Config.ToggleButton(tabName = "nodeSelector")
  public void setHaveGamePiece(boolean input) {
      this.haveGamePiece = input;
  }

  @Log.BooleanBox
  public boolean isLimitSwitchPressed() {
    return intakeHardStop.isPressed();
  }

  public Command waitUntilHaveGamePiece() {
    return Commands.waitUntil(new Trigger(() -> getIntakeCurrent() > 50).debounce(.5, DebounceType.kRising))
        .raceWith(Commands.waitUntil(this::isLimitSwitchPressed)).andThen(Commands.runOnce(() -> haveGamePiece = true));
  }
}
