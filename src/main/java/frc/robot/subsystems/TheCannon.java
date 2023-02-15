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
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.poi;
import frc.robot.RobotContainer.ArmTestSetPoints;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class TheCannon extends SubsystemBase implements Loggable {
  /** Creates a new TheCannon. */
public double extensionInches = 0;
public double testCannonAngle = 0;

private CANSparkMax cannonRotLead = new CANSparkMax(4, MotorType.kBrushless);
private CANSparkMax cannonRotFollow = new CANSparkMax(11, MotorType.kBrushless);
private CANSparkMax cannonExtension = new CANSparkMax(17, MotorType.kBrushless);

private AbsoluteEncoder cannonAbsolute = cannonRotLead.getAbsoluteEncoder(Type.kDutyCycle);
private RelativeEncoder extensionEncoder = cannonExtension.getEncoder();
private RelativeEncoder cannonRelative = cannonRotLead.getEncoder();

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
    cannonRelative.setPosition(Math.toRadians(-30));
    
    cannonExtension.setSmartCurrentLimit(70);
    
    cannonRelative.setPositionConversionFactor(Math.PI / 2 / 30.30927848815918);
    extensionEncoder.setPositionConversionFactor(1.802406002431152);
    
    cannonExtension.setInverted(true);
    
    cannonRotLead.getEncoder();
    
    cannonRotLead.setIdleMode(IdleMode.kBrake);
    cannonRotFollow.setIdleMode(IdleMode.kBrake);
    cannonExtension.setIdleMode(IdleMode.kBrake);
    
    cannonRotFollow.follow(cannonRotLead, true);
    
    cannonRotLeadPID.setP(Preferences.getDouble("CannonKP", 6.0 / Math.PI));
    cannonRotLeadPID.setI(Preferences.getDouble("CannonPI", 0.0));
    cannonRotLeadPID.setD(Preferences.getDouble("CannonKD", 0.0));
    
    cannonExtensionPID.setP(Preferences.getDouble("ExtensionKP", 1.0 / 6.0));
    cannonExtensionPID.setI(Preferences.getDouble("ExtensionKI", 0.0));
    cannonExtensionPID.setD(Preferences.getDouble("ExtensionKD", 0.0));
    
    // cannonAbsolute.setPositionConversionFactor(1/360);
    // cannonAbsolute.setVelocityConversionFactor(1/360);
    
    cannonRotLead.burnFlash();
    cannonRotFollow.burnFlash();
    cannonExtension.burnFlash();
    
    cannonRotLeadPID.setFeedbackDevice(cannonAbsolute); // Caleb note: don't think this needs to be run periodically,
    // only once
  }
  
  @Override
  public void periodic() {

    setAdaptiveFeedForward();

    cannonExtensionPID.setReference(extensionInches, ControlType.kPosition); 
    cannonRotLeadPID.setReference(testCannonAngle, ControlType.kPosition);

    if (Preferences.getBoolean("Wanna PID Cannon", false)) {
      cannonRotLeadPID.setP(Preferences.getDouble("CannonKP", 6.0 / Math.PI));
      cannonRotLeadPID.setI(Preferences.getDouble("CannonPI", 0.0));
      cannonRotLeadPID.setD(Preferences.getDouble("CannonKD", 0.0));

      cannonExtensionPID.setP(Preferences.getDouble("ExtensionKP", 1.0 / 6.0));
      cannonExtensionPID.setI(Preferences.getDouble("ExtensionKI", 0.0));
      cannonExtensionPID.setD(Preferences.getDouble("ExtensionKD", 0.0));
      Preferences.setBoolean("Wanna PID Cannon", false);
    }

  }
  // This method will be called once per scheduler run



  public void setAdaptiveFeedForward() {

    kS = 0;
    kA = (7.71E-3) * extensionEncoder.getPosition() - 0.133;
    kG = (0.0289) * extensionEncoder.getPosition() + (8.57E-3);
    kV = 2.44;
    m_feedforward = new ArmFeedforward(kS, kG, kV, kA);

  }

  public void stowMode() {
    cannonRotLead.set(0);
    cannonExtension.set(0);
    // cannonRotLeadPID.setReference(3, ControlType.kPosition);
    // cannonAbsolute.setPosition(3);

  }

  public void manExtend() {
    // extension motor spins
    cannonExtension.set(0.4);

  }

  public void manRetract() {
    // extension motor spins the other way
    cannonExtension.set(-0.4);

  }

  public void manRotUp() {
    // rotation motor spins
    cannonRotLead.set(0.4);

  }

  public void manRotDown() {
    // rotation motor spins the other way
    cannonRotLead.set(-0.4);

  }

  public void extendToSetpoint(poi poi) {
    setAdaptiveFeedForward();
    double ff = getArbitraryFeedForward();
    cannonRotLeadPID.setReference(poi.getCannonAngle(), ControlType.kPosition, 0, ff, ArbFFUnits.kVoltage);

  }

  public void rotateToSetpoint(poi poi) {
    setAdaptiveFeedForward();
    double ff = getArbitraryFeedForward();
    cannonExtensionPID.setReference(poi.getExtension(), ControlType.kPosition, 0, ff, ArbFFUnits.kVoltage);
  }

  public void setReference(double position) {
    setAdaptiveFeedForward();
    double ff = m_feedforward.calculate(Math.toRadians(cannonAbsolute.getPosition()),
        Math.toRadians(cannonAbsolute.getVelocity()));
    cannonExtensionPID.setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kVoltage);
  }

  @Log
  public double getCannonAngleEncoder() {
    return Math.toDegrees(cannonRelative.getPosition());
  }

  @Log
  public double getArbitraryFeedForward() {
    return m_feedforward.calculate(Math.toRadians(cannonAbsolute.getPosition()),
        Math.toRadians(cannonAbsolute.getVelocity()));
  }

  @Config(defaultValueBoolean = false)
  public void zeroArm(boolean input) {
    if (input) {
      cannonRelative.setPosition(0);
    }
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
    testCannonAngle = input;
    
  }
}
