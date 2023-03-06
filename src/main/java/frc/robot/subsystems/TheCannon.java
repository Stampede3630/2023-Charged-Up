// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CannonConstants;
import frc.robot.Constants.ExtendoConstants;
import frc.robot.RobotContainer.FacingPOI;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class TheCannon extends SubsystemBase implements Loggable {
  /** Creates a new TheCannon. */
@Log
public double extensionInches = ExtendoConstants.INITIALIZED_INCHES;
@Log
public double cannonRotation = CannonConstants.INITIALIZED_ANGLE;

private CANSparkMax cannonRotLead = new CANSparkMax(CannonConstants.SPARK_MASTER_ID, MotorType.kBrushless);
private CANSparkMax cannonRotFollow = new CANSparkMax(CannonConstants.SPARK_FOLLOWER_ID, MotorType.kBrushless);
private CANSparkMax cannonExtension = new CANSparkMax(ExtendoConstants.SPARK_MAX_ID, MotorType.kBrushless);

private AbsoluteEncoder cannonAbsolute = cannonRotLead.getAbsoluteEncoder(Type.kDutyCycle);
private RelativeEncoder extensionEncoder = cannonExtension.getEncoder();
private SparkMaxLimitSwitch extensionHardStop = cannonExtension.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

private SparkMaxPIDController cannonRotLeadPID = cannonRotLead.getPIDController();
private SparkMaxPIDController cannonExtensionPID = cannonExtension.getPIDController();

private ArmFeedforward m_feedforward =
  new ArmFeedforward(
    CannonConstants.KS, 
    CannonConstants.KG,
    CannonConstants.KV, 
    CannonConstants.KA);
  
  public TheCannon() {
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
    cannonRotLeadPID.setOutputRange(-1, 1);
    //cannonRotLeadPID.setOutputRange(-.5, .5); //KP Accounts for this I think
    
    cannonExtensionPID.setP(Preferences.getDouble("ExtensionKP", ExtendoConstants.KP));
    cannonExtensionPID.setI(Preferences.getDouble("ExtensionKI", ExtendoConstants.KI));
    cannonExtensionPID.setD(Preferences.getDouble("ExtensionKD", ExtendoConstants.KD));
    cannonExtensionPID.setOutputRange(-1, 1);
  
    
    cannonRotLead.burnFlash();
    cannonRotFollow.burnFlash();
    cannonExtension.burnFlash();
    
     
  }
  
  @Override
  public void periodic() {

    setAdaptiveFeedForward();

    cannonExtensionPID.setReference(extensionInches, ControlType.kPosition); 
    cannonRotLeadPID.setReference(getRevReferenceAngleSetpoint(), ControlType.kPosition, 0, getArbitraryFeedForward(), ArbFFUnits.kVoltage);

    if (Preferences.getBoolean("Wanna PID Cannon", false)) {
      cannonRotLeadPID.setP(Preferences.getDouble("CannonKP", CannonConstants.KP));
      cannonRotLeadPID.setI(Preferences.getDouble("CannonKI", CannonConstants.KI));
      cannonRotLeadPID.setD(Preferences.getDouble("CannonKD", CannonConstants.KD));

    cannonExtensionPID.setP(Preferences.getDouble("ExtensionKP", ExtendoConstants.KP));
    cannonExtensionPID.setI(Preferences.getDouble("ExtensionKI", ExtendoConstants.KI));
    cannonExtensionPID.setD(Preferences.getDouble("ExtensionKD", ExtendoConstants.KD));
      Preferences.setBoolean("Wanna PID Cannon", false);
    }

    setExtensionZero();

  }
  // This method will be called once per scheduler run


  //TODO: max extension 47inches, 0 is actually 13 inches
  public void setAdaptiveFeedForward() {
    double extensionPosition = extensionEncoder.getPosition();
    m_feedforward = 
      new ArmFeedforward(
        CannonConstants.KS,
        CannonConstants.KGM * extensionPosition + CannonConstants.KGB,
        CannonConstants.KV,
        CannonConstants.KAM * extensionPosition + CannonConstants.KAB
      );
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

  public void setCannonRotation(double intakeCannonAngle){
    setCannonAngle(intakeCannonAngle);

  }

  @Log
  public boolean getExtensionHardStop(){
    return extensionHardStop.isPressed();
  }

  public void setExtensionZero(){
    if (extensionHardStop.isPressed()){
      extensionEncoder.setPosition(0);
      extensionInches = 0;
    }
  }

  @Log
  public boolean cannonErrorWithinRange (){
    return Math.abs(cannonRotation - getCannonAngleEncoder()) < CannonConstants.ERROR; 
  }

  @Log
  public boolean extensionErrorWithinRange(){
    return Math.abs(extensionInches - getExtensionEncoder()) < ExtendoConstants.ERROR;
  }

  // public void stowMode() {
  //   cannonRotLead.set(0);
  //   cannonExtension.set(0);
  //   // cannonRotLeadPID.setReference(3, ControlType.kPosition);
  //   // cannonAbsolute.setPosition(3);

  // }

  public void manExtend() {
    // extension motor spins
    extensionInches += 1;

  }

  public void manRetract() {
    // extension motor spins the other way
    extensionInches -= 1;
  }

  public void manRotUp() {
    // rotation motor spins
    setCannonAngle(cannonRotation  + 1.0);

  }

  public void manRotDown() {
    setCannonAngle(cannonRotation  - 1.0);

  }

  /**
   * @return degress per sec
   */
  public double getCannonVelocity(){
    return cannonAbsolute.getVelocity() / 60.0;
  }

  @Log
  public double getCannonAngleEncoder() {
    return cannonAbsolute.getPosition() - 90;
  }

  @Log
  public double getArbitraryFeedForward() {
    return m_feedforward.calculate(Math.toRadians(getCannonAngleEncoder()),
        Math.toRadians(getCannonVelocity()));
  }

  @Log
  public double getExtensionEncoder(){
    return extensionEncoder.getPosition();
  }

  @Config
  public void setExtensionInches(double input){
    extensionInches = input;
  }
  @Config
  public void setCannonAngle(double input){
    cannonRotation = input;
  }

  public double setCannonAngleSides(FacingPOI robotFacing, double angle) {
    double angleToSet = cannonRotation; // no change
    if (robotFacing == FacingPOI.COMMUNITY)
      angleToSet = 180-angle;
    else if (robotFacing == FacingPOI.HUMAN_PLAYER)
      angleToSet = angle;

    setCannonAngle(angleToSet);
    return angleToSet;
  }

  public double getRevReferenceAngleSetpoint() {
     return (cannonRotation + 90);
  }
}
