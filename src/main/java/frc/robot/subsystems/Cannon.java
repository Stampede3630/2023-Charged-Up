// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CannonConstants;
import frc.robot.Constants.ExtendoConstants;
import frc.robot.RobotContainer.FacingPOI;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Cannon extends SubsystemBase implements Loggable {
  /** Creates a new TheCannon. */
public double extensionReference = ExtendoConstants.INITIALIZED_INCHES;
@Log
public double cannonReference = CannonConstants.INITIALIZED_ANGLE;

private final CANSparkMax cannonRotLead = new CANSparkMax(CannonConstants.SPARK_MASTER_ID, MotorType.kBrushless);
private final CANSparkMax cannonRotFollow = new CANSparkMax(CannonConstants.SPARK_FOLLOWER_ID, MotorType.kBrushless);
private final CANSparkMax cannonExtension = new CANSparkMax(ExtendoConstants.SPARK_MAX_ID, MotorType.kBrushless);

private final AbsoluteEncoder cannonAbsolute = cannonRotLead.getAbsoluteEncoder(Type.kDutyCycle);
private final RelativeEncoder extensionEncoder = cannonExtension.getEncoder();
private final SparkMaxLimitSwitch extensionHardStop = cannonExtension.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

private final SparkMaxPIDController cannonRotLeadPID = cannonRotLead.getPIDController();
private final SparkMaxPIDController cannonExtensionPID = cannonExtension.getPIDController();
private static Cannon instance;

  private Cannon() {
    cannonAbsolute.setInverted(false);
    cannonAbsolute.setPositionConversionFactor(CannonConstants.CONVERSION_FACTOR);
    cannonAbsolute.setVelocityConversionFactor(CannonConstants.CONVERSION_FACTOR);
    cannonAbsolute.setZeroOffset(CannonConstants.ZERO_OFFSET);

    cannonExtension.setSmartCurrentLimit(ExtendoConstants.CURRENT_LIMIT);
    cannonRotLead.setSmartCurrentLimit(CannonConstants.CURRENT_LIMIT);
    cannonRotFollow.setSmartCurrentLimit(CannonConstants.CURRENT_LIMIT);
    
    cannonExtension.setInverted(true);
    extensionEncoder.setPositionConversionFactor(ExtendoConstants.MAGIC_TO_INCHES);
    
    // cannonExtension.setInverted(true);
    //changed idle mode to help with troubleshooting    
    cannonRotLead.setIdleMode(IdleMode.kCoast); //I 
    cannonRotFollow.setIdleMode(IdleMode.kCoast);
    cannonExtension.setIdleMode(IdleMode.kCoast);
    cannonRotFollow.follow(cannonRotLead, false);
    cannonRotLead.setSoftLimit(SoftLimitDirection.kForward, CannonConstants.FORWARD_LIMIT);
    cannonRotLead.setSoftLimit(SoftLimitDirection.kReverse, CannonConstants.REVERSE_LIMIT);
    
    cannonRotLeadPID.setFeedbackDevice(cannonAbsolute);
    cannonRotLeadPID.setPositionPIDWrappingEnabled(false);

    cannonRotLeadPID.setP(Preferences.getDouble("CannonKP", CannonConstants.KP));
    cannonRotLeadPID.setI(Preferences.getDouble("CannonKI", CannonConstants.KI));
    cannonRotLeadPID.setD(Preferences.getDouble("CannonKD", CannonConstants.KD));
    cannonRotLeadPID.setOutputRange(-.8, .8);

    //cannonRotLeadPID.setOutputRange(-.5, .5); //P Accounts for this I thinkK
    
    cannonExtensionPID.setP(Preferences.getDouble("ExtensionKP", ExtendoConstants.KP));
    cannonExtensionPID.setI(Preferences.getDouble("ExtensionKI", ExtendoConstants.KI));
    cannonExtensionPID.setD(Preferences.getDouble("ExtensionKD", ExtendoConstants.KD));
    cannonExtensionPID.setOutputRange(-.5, 1);

    cannonExtension.setSoftLimit(SoftLimitDirection.kForward, 40);
    cannonExtension.setSoftLimit(SoftLimitDirection.kReverse, 0);
    extensionHardStop.enableLimitSwitch(true);
    
    
    cannonRotLead.burnFlash();
    cannonRotFollow.burnFlash();
    cannonExtension.burnFlash();


  }

  public static Cannon getInstance() {
    if (instance == null)
      instance = new Cannon();
    return instance;
  }

  public double extendoTemp() {
    return cannonExtension.getMotorTemperature();
  }

  public double extendoCurrent() {
    return cannonExtension.getOutputCurrent();
  }
  
  @Override
  public void periodic() {

    if (Preferences.getBoolean("Wanna PID Cannon", false)) {
      cannonRotLeadPID.setP(Preferences.getDouble("CannonKP", CannonConstants.KP));
      cannonRotLeadPID.setI(Preferences.getDouble("CannonKI", CannonConstants.KI));
      cannonRotLeadPID.setD(Preferences.getDouble("CannonKD", CannonConstants.KD));

    cannonExtensionPID.setP(Preferences.getDouble("ExtensionKP", ExtendoConstants.KP));
    cannonExtensionPID.setI(Preferences.getDouble("ExtensionKI", ExtendoConstants.KI));
    cannonExtensionPID.setD(Preferences.getDouble("ExtensionKD", ExtendoConstants.KD));
      Preferences.setBoolean("Wanna PID Cannon", false);
    }
    if (getExtensionHardStop()){
      extensionEncoder.setPosition(0);
      setExtensionReference(0);
    }
  }
  
  @Log(tabName = "nodeSelector")
  public double getExtensionReference(){
    return extensionReference;
  }

  public void setCannonToCoast(){
    cannonRotLead.setIdleMode(IdleMode.kCoast);
    cannonRotFollow.setIdleMode(IdleMode.kCoast);
    cannonExtension.setIdleMode(IdleMode.kCoast);
  }

  public void setCannonToBrake(){
    cannonRotLead.setIdleMode(IdleMode.kBrake);
    cannonRotFollow.setIdleMode(IdleMode.kBrake);
    cannonExtension.setIdleMode(IdleMode.kBrake);
  }

  public boolean getExtensionHardStop(){
    return extensionHardStop.isPressed();
  }

  public boolean cannonErrorWithinRange (){
    return Math.abs(cannonReference - getCannonAngleEncoder()) < CannonConstants.ERROR;
  }

  public boolean extensionErrorWithinRange(){
    return Math.abs(extensionReference - getExtensionEncoder()) < ExtendoConstants.ERROR;
  }

  public void manExtend() {
    // extension motor spins
    setExtensionReference(extensionReference + 1);
  }

  public void manRetract() {
    // extension motor spins the other way
    setExtensionReference(extensionReference - 1);
  }

  public void manRotUp() {
    // rotation motor spins
    setCannonAngle(cannonReference + 1.0);

  }

  public void manRotDown() {
    setCannonAngle(cannonReference - 1.0);

  }

  /**
   * @return degrees per sec
   */
  public double getCannonVelocity(){
    return cannonAbsolute.getVelocity() / 60.0;
  }

  @Log
  public double getCannonAngleEncoder() {
    return cannonAbsolute.getPosition() - 90;
  }

  //TODO: max extension 47inches, 0 is actually 13 inches
  public double getArbitraryFeedForward() {
    double extensionPosition = extensionEncoder.getPosition();
    ArmFeedforward feedforward = new ArmFeedforward(
            CannonConstants.KS,
            CannonConstants.KGM * extensionPosition + CannonConstants.KGB,
            CannonConstants.KV,
            CannonConstants.KAM * extensionPosition + CannonConstants.KAB
    );
    return feedforward.calculate(Math.toRadians(getCannonAngleEncoder()),
        Math.toRadians(getCannonVelocity()));
  }

  @Log
  public double getExtensionEncoder(){
    return extensionEncoder.getPosition();
  }

  @Config.NumberSlider(max=45,min=0)
  public void setExtensionReference(double input){
    extensionReference = input;
    cannonExtensionPID.setReference(extensionReference, ControlType.kPosition);
  }
  @Config.NumberSlider(max = 230, min = -20, defaultValue = CannonConstants.INITIALIZED_ANGLE)
  public void setCannonAngle(double input){
    cannonReference = input;
    cannonRotLeadPID.setReference(getRevReferenceAngleSetpoint(), ControlType.kPosition, 0, getArbitraryFeedForward(), ArbFFUnits.kVoltage);
  }


  public Command setCannonAngleWait(DoubleSupplier angleGetter) {
    return new FunctionalCommand(
      () -> setCannonAngle(angleGetter.getAsDouble()), 
      () -> {},
      (success) -> {System.out.println("rotate done");}, 
      this::cannonErrorWithinRange);
  }
  public Command setExtensionWait(DoubleSupplier inchesGetter) {
    return new FunctionalCommand(
      () -> setExtensionReference(inchesGetter.getAsDouble()), 
      () -> {},
      (success) -> {System.out.println("extend done");},
      this::extensionErrorWithinRange);
  }
  public double setCannonAngleSides(FacingPOI robotFacing, double angle) {
    double angleToSet = cannonReference; // no change
    if (robotFacing == FacingPOI.COMMUNITY)
      angleToSet = 180-angle;
    else if (robotFacing == FacingPOI.HUMAN_PLAYER)// hp or nothing
      angleToSet = angle;

    setCannonAngle(angleToSet);
    return angleToSet;
  }

  public double getRevReferenceAngleSetpoint() {
     return (cannonReference + 90);
  }
}
