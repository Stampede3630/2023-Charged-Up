package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Lid;
import frc.robot.subsystems.Lid.LidPosition;
import frc.robot.util.Disableable;
import frc.robot.util.Enableable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable, Disableable, Enableable {
    private final TalonFX m_intakeMotor = new TalonFX(14, "rio");
    private final SparkMaxLimitSwitch intakeHardStop = Lid.getInstance().getReverseLimitSwitch();
    private double speed = 0;
    private boolean haveGamePiece = false;
    private final LinearFilter currentFilter = LinearFilter.movingAverage(10);
    private final double A_LITLE_AMOUNT = 1;
    @Log
    private double filteredCurrent;
    private final Debouncer m_debouncer = new Debouncer(0.30, DebounceType.kRising);
    public Intake() {
         m_intakeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 80, 70, .05));
         m_intakeMotor.setNeutralMode(NeutralMode.Coast);
        // cannonExtension.setInverted(true);
        //changed idle mode to help with troubleshooting    
    }

    @Override
    public void onDisable() {
        m_intakeMotor.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void onEnable() {
        m_intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        m_intakeMotor.set(ControlMode.PercentOutput,speed);
        filteredCurrent = currentFilter.calculate(getIntakeCurrent());
    }

    public void setIntake(double intakeSpeed) {
        speed = intakeSpeed;
    }

    public void stopIntake() {
        speed = 0;
    }

    @Log
    public double getMotorTemp() {
        return m_intakeMotor.getTemperature();
    }

    @Log.Graph
    public double getIntakeCurrent() {
        return m_intakeMotor.getStatorCurrent();
    }

    @Log
    public double getVoltage() {
        return m_intakeMotor.getMotorOutputVoltage();
    }

    /**
     * gets the encoder position in rotations
     * @return encoder position in rot
     */
    @Log
    public double getEncoderPosition() {
        return m_intakeMotor.getSelectedSensorPosition()/2048;
    }

    /**
     * sets the encoder position in rotations
     * @param position the new position in rotations
     */
    public void setEncoderPosition(double position) {
        m_intakeMotor.setSelectedSensorPosition(position*2048);
    }

    @Log.BooleanBox(tabName = "nodeSelector")
    public boolean haveGamePiece() {
        return haveGamePiece;
    }

    public void leaveGamePiece(){
        haveGamePiece = false;
    }

    public Command inALittle(double intakeSpeed, LidPosition lidPosition){
        double intakeSign = Math.copySign(1, intakeSpeed);
        return new FunctionalCommand(
                () -> setEncoderPosition(0),   // first zero the encoder
                () -> {// then run the intake
                    if (lidPosition == LidPosition.TOP)
                        setIntake(intakeSign*.1); // bc intake speed is always on the HP side
                    else 
                        setIntake(-intakeSign*.1);
                },
                (success) -> setIntake(0),      // when finished, stop the intake
                () -> {
                    if (lidPosition == LidPosition.TOP)
                        return (intakeSign >= 0) ? getEncoderPosition() >= A_LITLE_AMOUNT :getEncoderPosition() <= -A_LITLE_AMOUNT;
                    else
                        return (intakeSign >= 0) ? getEncoderPosition() <= -A_LITLE_AMOUNT :getEncoderPosition() >= A_LITLE_AMOUNT;
                }); // it is finished when the encoder goes down by 15 rotations
    }
    public Command outALittle(double intakeSpeed, LidPosition lidPosition){
        double intakeSign = Math.copySign(1, intakeSpeed);
        return new FunctionalCommand(
                () -> setEncoderPosition(0),   // first zero the encoder
                () -> {// then run the intake
                    if (lidPosition == LidPosition.TOP)
                        setIntake(-intakeSign*.1); // bc intake speed is always on the HP side
                    else 
                        setIntake(intakeSign*.1);
                },            
                (success) -> setIntake(0),      // when finished, stop the intake
                () -> {
                    if (lidPosition == LidPosition.TOP)
                        return (intakeSign >= 0) ? getEncoderPosition() <= -A_LITLE_AMOUNT : getEncoderPosition() >= A_LITLE_AMOUNT;
                    else
                        return (intakeSign >= 0) ? getEncoderPosition() >= A_LITLE_AMOUNT : getEncoderPosition() <= -A_LITLE_AMOUNT;

                }); // it is finished when the encoder goes down by 15 rotations
    }
    public boolean checkForGamePiece() {
        haveGamePiece = haveGamePiece || intakeHardStop.isPressed(); // latching
        return haveGamePiece;
    }
  @Config.ToggleButton(tabName = "nodeSelector")
  public void setHaveGamePiece(boolean input) {
      this.haveGamePiece = input;
  }

  @Log.BooleanBox(tabName = "nodeSelector")
  public boolean isLimitSwitchPressed() {
    return intakeHardStop.isPressed();
  }

    /**
     * gets the velocity of the intake motor (rot/s)
     * @return velocity of the intake motor in rot/s
     */
  public double getVelocity() {
    return m_intakeMotor.getSelectedSensorVelocity()/2048*10;
  }

  public Command waitUntilHaveGamePiece() {
    return Commands.waitUntil(()-> Math.abs(getVelocity()) > 15) // wait for spin up
            .andThen(Commands.waitUntil(() -> m_debouncer.calculate(Math.abs(getVelocity()) < 5) || isLimitSwitchPressed())) // wait for stopped TODO adjust the numbers
            .andThen(Commands.runOnce(() -> haveGamePiece = true));
  }
}
