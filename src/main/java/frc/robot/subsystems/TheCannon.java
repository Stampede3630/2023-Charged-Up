// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.node;

public class TheCannon extends SubsystemBase {
  /** Creates a new TheCannon. */
  public CANSparkMax cannonRotLead = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax cannonRotFollow = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax extensionMotor = new CANSparkMax(3, MotorType.kBrushless);
  
  public TheCannon() {
  }

  @Override
  public void periodic() {
    cannonRotFollow.follow(cannonRotLead);
    // This method will be called once per scheduler run
  }

  public void zeroCannon(){
    //TODO: decide where rotation zero should be

  }

  public void zeroExtension(){
    //fully retract arm using absolute encoder/limit switch or smthng
  }

  public void manExtend(){
    //extension motor spins
    extensionMotor.set(0.5);

  }

  public void manRetract(){
    //extension motor spins the other way
    extensionMotor.set(-0.5);

  }

  public void manRotUp(){

    cannonRotLead.set(0.5);

    //rotation motor spins

  }

  public void manRotDown(){
    //rotation motor spins the other way
    cannonRotLead.set(-0.5);

  }

  public void extendToSetpoint(node node){
    //take extension measurement from table (TODO: Make Table) and pipe it in here
    //here we say something like "motor.setPosition(node.getExtension)"
    // MotorController.setPosition(node.getExtension()); like this -ej

  }

  public void rotateToSetpoint(node node){
    //take rotation measurement from table and pipe it in here
    //here we say something like "motor.setPosition(node.getCannonAngle)"
    //MotorController.setAngle(node.getCannonAngle()); like this -ej

  }

}
