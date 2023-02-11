// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePieceOrientation;


public class RotoClawtake extends SubsystemBase {

  public double rotoMeasure;

  public Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
  public DoubleSolenoid clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
  public CANSparkMax clawTakeMotor = new CANSparkMax(14, MotorType.kBrushless);
  public CANSparkMax rotoMotor = new CANSparkMax(13, MotorType.kBrushless);
  public CANSparkMax clawMotor = new CANSparkMax(0, MotorType.kBrushless);

  public DigitalInput clawSwitch = new DigitalInput(0);


  public SparkMaxPIDController rotoMotorPID = rotoMotor.getPIDController();
  public SparkMaxPIDController clawMotorPID = clawMotor.getPIDController();

  /** Creates a new Claw. */
  public RotoClawtake() {
    compressor.enableDigital();    

    clawMotorPID.setP(1);
    clawMotorPID.setI(0);
    clawMotorPID.setD(0.1);

    rotoMotorPID.setP(1);
    rotoMotorPID.setI(0);
    rotoMotorPID.setD(0.1);

    rotoMotor.burnFlash();
    clawTakeMotor.burnFlash();

    

    // compressor.enableDigital();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void stopClawTake(){
    clawTakeMotor.set(0);
    rotoMotor.set(0);
  }

  public void openClaw(){
    //open piston
    // clawSolenoid.set(Value.kForward);
    clawMotorPID.setReference(4, ControlType.kPosition);
  }

  public void closeClaw(){
    //close piston
    // clawSolenoid.set(Value.kReverse);
    clawMotorPID.setReference(0, ControlType.kPosition);
    
  }

  public void runClawtake(){
    //run the rotoclawtake/grab a game piece
    clawTakeMotor.set(-1);
  }
  public void reverseClawtake(){
    //spit out a game piece/outtake rotoclawtake
    clawTakeMotor.set(1);
  }

  public void rotoClaw(GamePieceOrientation gamePieceOrientation){
    //twist the rotoclawtake
    // rotoMotorPID.setReference(gamePieceOrientation.getRotOrientForRoto(), ControlType.kPosition);
    rotoMotor.set(0.2);
  }

  public void actuateClaw(GamePieceOrientation gamePieceOrientation){
    clawMotorPID.setReference(gamePieceOrientation.getCone(gamePieceOrientation), ControlType.kPosition);
    
  }
  

}
