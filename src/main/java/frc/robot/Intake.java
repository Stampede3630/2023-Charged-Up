package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable{
    private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.SPARK_MAX_ID, MotorType.kBrushless);
    private final RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();
    private final SparkMaxPIDController m_intakePid = m_intakeMotor.getPIDController();
    private final SparkMaxLimitSwitch intakeHardStop = m_intakeMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    private double speed = 0;
    private boolean haveGamePiece = false;
    @Log
    private double intakeEncoderPosition;
    public Intake() {

        m_intakeMotor.setSmartCurrentLimit(80);

                
        // cannonExtension.setInverted(true);
        //changed idle mode to help with troubleshooting    
        m_intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeHardStop.enableLimitSwitch(true); 

        m_intakeMotor.burnFlash();
        m_intakeEncoder.setPositionConversionFactor(1); // TODO Put some numbers here
        m_intakeEncoder.setVelocityConversionFactor(1);


    }

    @Override
    public void periodic() {
        m_intakeMotor.set(speed);
        intakeEncoderPosition = m_intakeEncoder.getPosition();
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

    public Command outALittle(){
        return new FunctionalCommand(
                () -> m_intakeEncoder.setPosition(0),   // first zero the encoder
                () -> setIntake(-.1),            // then run the intake
                (success) -> setIntake(0),      // when finished, stop the intake
                () -> m_intakeEncoder.getPosition() <= -15); // it is finished when the encoder goes down by 15 rotations
    }
    public Command inALittle(){
        return new FunctionalCommand(
                () -> m_intakeEncoder.setPosition(0),   // first zero the encoder
                () -> setIntake(.1),            // then run the intake
                (success) -> setIntake(0),      // when finished, stop the intake
                () -> m_intakeEncoder.getPosition() >= 15); // it is finished when the encoder goes up by 15 rotations
    }
    public boolean checkForGamePiece() {
        haveGamePiece = haveGamePiece || intakeHardStop.isPressed(); // latching
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
    return Commands.waitUntil(new Trigger(() -> getIntakeCurrent() > 40).debounce(.5, DebounceType.kRising))
        .raceWith(Commands.waitUntil(this::isLimitSwitchPressed)).andThen(Commands.runOnce(() -> haveGamePiece = true));
  }
}
