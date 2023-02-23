// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.FacingPOI;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class TheCannon extends SubsystemBase implements Loggable {
  /** Creates a new TheCannon. */
@Log
public double extensionInches = 0.0;
@Log
public double cannonRotation = 0.0;

private CANSparkMax cannonRotLead = new CANSparkMax(4, MotorType.kBrushless);
private CANSparkMax cannonRotFollow = new CANSparkMax(11, MotorType.kBrushless);
private CANSparkMax cannonExtension = new CANSparkMax(17, MotorType.kBrushless);

private AbsoluteEncoder cannonAbsolute = cannonRotLead.getAbsoluteEncoder(Type.kDutyCycle);
private RelativeEncoder extensionEncoder = cannonExtension.getEncoder();
private SparkMaxLimitSwitch extensionHardStop = cannonExtension.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

private SparkMaxPIDController cannonRotLeadPID = cannonRotLead.getPIDController();
private SparkMaxPIDController cannonExtensionPID = cannonExtension.getPIDController();

private double kS = 0;
private double kG = 0.73;
private double kV = 2.44;
private double kA = 0.06;

private ArmFeedforward m_feedforward =

new ArmFeedforward(
  kS, kG,
  kV, kA);
  
  public TheCannon() {
    cannonAbsolute.setInverted(false);
    cannonAbsolute.setPositionConversionFactor(360.0);
    cannonAbsolute.setVelocityConversionFactor(360.0);
    cannonAbsolute.setZeroOffset(66.36);

    cannonExtension.setSmartCurrentLimit(70);
    
    extensionEncoder.setPositionConversionFactor(1.802406002431152);
    
    cannonExtension.setInverted(true);
    //changed idle mode to help with troubleshooting    
    cannonRotLead.setIdleMode(IdleMode.kBrake);
    cannonRotFollow.setIdleMode(IdleMode.kBrake);
    cannonExtension.setIdleMode(IdleMode.kBrake);
    cannonRotFollow.follow(cannonRotLead, true);

    cannonRotLeadPID.setFeedbackDevice(cannonAbsolute);
    cannonRotLeadPID.setPositionPIDWrappingEnabled(false);

    cannonRotLeadPID.setP(Preferences.getDouble("CannonKP", 1.0/30.0));
    cannonRotLeadPID.setI(Preferences.getDouble("CannonPI", 0.0));
    cannonRotLeadPID.setD(Preferences.getDouble("CannonKD", 0.0));
    cannonRotLeadPID.setOutputRange(-.5, .5);
    
    cannonExtensionPID.setP(Preferences.getDouble("ExtensionKP", 1.0 / 6.0));
    cannonExtensionPID.setI(Preferences.getDouble("ExtensionKI", 0.0));
    cannonExtensionPID.setD(Preferences.getDouble("ExtensionKD", 0.0));
    


    
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
      cannonRotLeadPID.setP(Preferences.getDouble("CannonKP", 1.0/30.0));
      cannonRotLeadPID.setI(Preferences.getDouble("CannonPI", 0.0));
      cannonRotLeadPID.setD(Preferences.getDouble("CannonKD", 0.0));

      cannonExtensionPID.setP(Preferences.getDouble("ExtensionKP", 1.0 / 6.0));
      cannonExtensionPID.setI(Preferences.getDouble("ExtensionKI", 0.0));
      cannonExtensionPID.setD(Preferences.getDouble("ExtensionKD", 0.0));
      Preferences.setBoolean("Wanna PID Cannon", false);
    }

    setExtensionZero();

  }
  // This method will be called once per scheduler run


  //TODO: max extension 47inches, 0 is actually 13 inches
  public void setAdaptiveFeedForward() {

    kS = 0;
    kA = (7.71E-3) * extensionEncoder.getPosition() - 0.133;
    kG = (0.0289) * extensionEncoder.getPosition() + (8.57E-3);
    kV = 2.44;
    m_feedforward = new ArmFeedforward(kS, kG, kV, kA);

  }

  @Log
  public boolean getExtensionHardStop(){
    return extensionHardStop.isPressed();
  }

  public void setExtensionZero(){
    if (extensionHardStop.isPressed()){
      extensionEncoder.setPosition(0);
    }
  }

  @Log
  public boolean cannonErrorWithinRange (){
    return Math.abs(cannonRotation - getCannonAngleEncoder()) < 10.0 ? true:false; 
  }

  @Log
  public boolean extensionErrorWithinRange(){
    return Math.abs(extensionInches - getExtensionEncoder()) < 5.0 ? true:false;
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
