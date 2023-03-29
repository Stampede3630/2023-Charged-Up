package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.ReverseLimitValue;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Lid.LidPosition;
import frc.robot.util.Disableable;
import frc.robot.util.Enableable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable, Disableable, Enableable {
    private final TalonFX m_intakeMotor;
    private final SparkMaxLimitSwitch intakeHardStop = Lid.getInstance().getReverseLimitSwitch();
    private double speed = 0;
    private boolean haveGamePiece = false;
    private final LinearFilter currentFilter = LinearFilter.movingAverage(10);
    private final double A_LITLE_AMOUNT = 1;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    @Config
    public double MAX_SPEED_RPS = 75;
    private double filteredCurrent;
    private final Debouncer m_debouncer = new Debouncer(0.1, DebounceType.kRising);
    private static Intake instance;
    private Intake() {
        m_intakeMotor = new TalonFX(14, "rio");
        m_intakeMotor.getConfigurator().refresh(motorConfig);
        motorConfig.CurrentLimits.StatorCurrentLimit = 80;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.Slot0.kP = 5;
        motorConfig.Slot0.kI= 0;
        motorConfig.Slot0.kD = 0;


        m_intakeMotor.getConfigurator().apply(motorConfig);
        // cannonExtension.setInverted(true);
        //changed idle mode to help with troubleshooting    
    }

    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }

    @Override
    public void onDisable() {
        m_intakeMotor.getConfigurator().refresh(motorConfig);
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_intakeMotor.getConfigurator().apply(motorConfig);
        System.out.println("disabled intake");
    }

    @Override
    public void onEnable() {
        m_intakeMotor.getConfigurator().refresh(motorConfig);
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_intakeMotor.getConfigurator().apply(motorConfig);
        System.out.println("enabled intake");

    }

    @Override
    public void periodic() {
        m_intakeMotor.setControl(new VelocityTorqueCurrentFOC(MAX_SPEED_RPS*speed, 0, 0, false));
        filteredCurrent = currentFilter.calculate(getIntakeCurrent());
    }

    public void setIntake(double intakeSpeed) {
        speed = intakeSpeed;
    }

    public void stopIntake() {
        speed = 0;
    }

    public double getMotorTemp() {
        return m_intakeMotor.getDeviceTemp().getValue();
    }

    public double getIntakeCurrent() {
        return m_intakeMotor.getStatorCurrent().getValue();
    }

    /**
     * gets the encoder position in rotations
     * @return encoder position in rot
     */
    @Log
    public double getEncoderPosition() {
        return m_intakeMotor.getRotorPosition().getValue();
    }
    /**
     * sets the encoder position in rotations
     * @param position the new position in rotations
     */
    public void setEncoderPosition(double position) {
        m_intakeMotor.setRotorPosition(position);
    }

    @Log.BooleanBox(tabName = "nodeSelector")
    public boolean haveGamePiece() {
        return haveGamePiece;
    }

    public void leaveGamePiece(){
        haveGamePiece = false;
    }

    public Command inALittle(DoubleSupplier intakeSpeed, Supplier<LidPosition> lidPosition){
        // double intakeSign = Math.copySign(1, intakeSpeed);
        // return new FunctionalCommand(
        //         () -> setEncoderPosition(0),   // first zero the encoder
        //         () -> {// then run the intake
        //             if (lidPosition == LidPosition.TOP)
        //                 setIntake(intakeSign*.1); // bc intake speed is always on the HP side
        //             else 
        //                 setIntake(-intakeSign*.1);
        //         },
        //         (success) -> setIntake(0),      // when finished, stop the intake
        //         () -> {
        //             if (lidPosition == LidPosition.TOP)
        //                 return (intakeSign >= 0) ? getEncoderPosition() >= A_LITLE_AMOUNT :getEncoderPosition() <= -A_LITLE_AMOUNT;
        //             else
        //                 return (intakeSign >= 0) ? getEncoderPosition() <= -A_LITLE_AMOUNT :getEncoderPosition() >= A_LITLE_AMOUNT;
        //         }); // it is finished when the encoder goes down by 15 rotations
        return Commands.none();
    }
    public Command outALittle(DoubleSupplier intakeSpeed, Supplier<LidPosition> lidPosition){
        return Commands.none();

        // double intakeSign = Math.copySign(1, intakeSpeed);
        // return new FunctionalCommand(
        //         () -> setEncoderPosition(0),   // first zero the encoder
        //         () -> {// then run the intake
        //             if (lidPosition == LidPosition.TOP)
        //                 setIntake(-intakeSign*.1); // bc intake speed is always on the HP side
        //             else 
        //                 setIntake(intakeSign*.1);
        //         },            
        //         (success) -> setIntake(0),      // when finished, stop the intake
        //         () -> {
        //             if (lidPosition == LidPosition.TOP)
        //                 return (intakeSign >= 0) ? getEncoderPosition() <= -A_LITLE_AMOUNT : getEncoderPosition() >= A_LITLE_AMOUNT;
        //             else
        //                 return (intakeSign >= 0) ? getEncoderPosition() >= A_LITLE_AMOUNT : getEncoderPosition() <= -A_LITLE_AMOUNT;

                // }); // it is finished when the encoder goes down by 15 rotations
    }
    public boolean checkForCube() {
        haveGamePiece = haveGamePiece || intakeHardStop.isPressed(); // latching
        return haveGamePiece;
    }
  @Config.ToggleButton(tabName = "nodeSelector")
  public void setHaveGamePiece(boolean input) {
      this.haveGamePiece = input;
  }

  @Log.BooleanBox(tabName = "nodeSelector")
  public boolean isLimitSwitchPressed() {

    return m_intakeMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

    /**
     * gets the velocity of the intake motor (rot/s)
     * @return velocity of the intake motor in rot/s
     */
  public double getVelocity() {
    return m_intakeMotor.getRotorVelocity().getValue();
  }
  
  public Command waitUntilHaveGamePiece(BooleanSupplier isItACube) {
    if (isItACube.getAsBoolean()) {
        return Commands.waitUntil(()-> Math.abs(getVelocity()) > 50)//.raceWith(Commands.waitSeconds(1)) // wait for spin up
            .andThen(Commands.waitUntil(() -> m_debouncer.calculate(isLimitSwitchPressed()))) // wait for stopped TODO adjust the numbers
            .andThen(Commands.runOnce(() -> haveGamePiece = true))  ;
    } else 
        return Commands.waitUntil(()-> Math.abs(getVelocity()) > 50)//.raceWith(Commands.waitSeconds(1)) // wait for spin up
                .andThen(Commands.waitUntil(() -> m_debouncer.calculate(Math.abs(getVelocity()) < 5))) // wait for stopped TODO adjust the numbers
                .andThen(Commands.runOnce(() -> haveGamePiece = true))  ;
  }


}
