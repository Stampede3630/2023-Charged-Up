// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.poi;

public class TheCannon extends SubsystemBase {
  /** Creates a new TheCannon. */
  private CANSparkMax cannonRotLead = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax cannonRotFollow = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax cannonExtension = new CANSparkMax(3, MotorType.kBrushless);

  // public DutyCycleEncoder cannonAbsolute = new DutyCycleEncoder(0);

  private RelativeEncoder cannonAbsolute = cannonRotLead.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192); //TODO FIX

  private SparkMaxPIDController cannonRotLeadPID = cannonRotLead.getPIDController();
  private SparkMaxPIDController cannonExtensionPID = cannonExtension.getPIDController();

  private double kS = 0;
  private double kG = 0;
  private double kV = 0;
  private double kA = 0;

  private final ArmFeedforward m_feedforward = 

  new ArmFeedforward(
      kS, kG,
      kV, kA);
  
  public TheCannon() {
    cannonRotFollow.follow(cannonRotLead);

    cannonRotLeadPID.setP(1);
    cannonRotLeadPID.setI(0);
    cannonRotLeadPID.setD(0.1);

    cannonExtensionPID.setP(1);
    cannonExtensionPID.setI(0);
    cannonExtensionPID.setD(0.1);

    cannonAbsolute.setPositionConversionFactor(1/360);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    cannonRotLeadPID.setFeedbackDevice(cannonAbsolute); // Caleb note: don't think this needs to be run periodically, only once
    
  }

  public void setAdaptiveFeedForward(){
    // kS = 
    // kG = 
    // kV = 
    // kA = 
  }

  public void stowMode(){

    cannonRotLeadPID.setReference(3, ControlType.kPosition);
    // cannonAbsolute.setPosition(3);

  }

  public void manExtend(){
    //extension motor spins
    cannonExtension.set(0.1);

  }

  public void manRetract(){
    //extension motor spins the other way
    cannonExtension.set(-0.1);

  }

  public void manRotUp(){
    //rotation motor spins
    cannonRotLead.set(0.5);

  }

  public void manRotDown(){
    //rotation motor spins the other way
    cannonRotLead.set(-0.5);

  }

  public void extendToSetpoint(poi poi){

    cannonRotLeadPID.setReference(poi.getCannonAngle(), ControlType.kPosition);

  }

  public void rotateToSetpoint(poi poi){

    cannonExtensionPID.setReference(poi.getExtension(), ControlType.kPosition);

  }
}
