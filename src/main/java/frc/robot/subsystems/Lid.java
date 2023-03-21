package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LidConstants;
import frc.robot.RobotContainer.FacingPOI;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Lid extends SubsystemBase implements Loggable{
    private final CANSparkMax m_lidMotor = new CANSparkMax(LidConstants.SPARK_MAX_ID, MotorType.kBrushless);
    private final SparkMaxAbsoluteEncoder m_lidAbsolute = m_lidMotor.getAbsoluteEncoder(Type.kDutyCycle);
    private final SparkMaxPIDController m_lidPid = m_lidMotor.getPIDController();
    private static Lid instance;
    @Log(tabName = "NodeSelector")
    public double lidReference = LidConstants.INITIALIZED_ANGLE;

    private Lid() {
        m_lidAbsolute.setInverted(true);
        m_lidAbsolute.setPositionConversionFactor(LidConstants.CONVERSION_FACTOR);
        m_lidAbsolute.setVelocityConversionFactor(LidConstants.CONVERSION_FACTOR);
        m_lidMotor.setSoftLimit(SoftLimitDirection.kForward, LidConstants.FORWARD_LIMIT);
        m_lidMotor.setSoftLimit(SoftLimitDirection.kReverse, LidConstants.REVERSE_LIMIT);
        m_lidAbsolute.setZeroOffset(LidConstants.ZERO_OFFSET);

        m_lidAbsolute.setPositionConversionFactor(360);
        m_lidAbsolute.setVelocityConversionFactor(360);

        m_lidMotor.setSmartCurrentLimit(LidConstants.CURRENT_LIMIT);
                
        // cannonExtension.setInverted(true);
        //changed idle mode to help with troubleshooting    
        m_lidMotor.setIdleMode(IdleMode.kBrake); 
        m_lidMotor.setInverted(true);

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

    
    @Config.NumberSlider(defaultValue = LidConstants.INITIALIZED_ANGLE, max = 260, min = 35, blockIncrement = 1)
    public void setLidReference(double input) {
        this.lidReference = input;
    }

    @Log(tabName = "NodeSelector")
    public double lidPosition() {
        return m_lidAbsolute.getPosition();
    }
    
    public double getLidCurrent() {
        return m_lidMotor.getOutputCurrent();
    }

    public boolean lidWithinError(){
        return Math.abs(lidReference - lidPosition()) < LidConstants.ERROR;
    }

    public SparkMaxLimitSwitch getReverseLimitSwitch() {
        return m_lidMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    }

    public static Lid getInstance() {
        if (instance == null)
            instance = new Lid();
        return instance;

    }

    public enum LidPosition {
        TOP, BOTTOM, UNKNOWN;

        public static LidPosition getLidPosition(FacingPOI robotFacing, FacingPOI cannonFacing) {
            if (robotFacing==FacingPOI.HUMAN_PLAYER && cannonFacing == FacingPOI.HUMAN_PLAYER)
                return TOP;
            else if (robotFacing==FacingPOI.HUMAN_PLAYER && cannonFacing == FacingPOI.COMMUNITY)
                return BOTTOM;
            else if (robotFacing==FacingPOI.COMMUNITY && cannonFacing == FacingPOI.COMMUNITY)
                return TOP;
            else if (robotFacing==FacingPOI.COMMUNITY && cannonFacing == FacingPOI.HUMAN_PLAYER)
                return BOTTOM;
            else
                return UNKNOWN;
        }
    }
}
