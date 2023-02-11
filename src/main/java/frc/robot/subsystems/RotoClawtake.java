// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePieceOrientation;
import frc.robot.RobotContainer.GamePieceType;


public class RotoClawtake extends SubsystemBase {

  public double rotoMeasure;

  public DoubleSolenoid clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  public CANSparkMax clawTakeMotor = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax rotoMotor = new CANSparkMax(6, MotorType.kBrushless);

  public SparkMaxPIDController rotoMotorPID = rotoMotor.getPIDController();

  /** Creates a new Claw. */
  public RotoClawtake() {
    rotoMotorPID.setP(1);
    rotoMotorPID.setI(0);
    rotoMotorPID.setD(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void openClaw(){
    //open piston
    clawSolenoid.set(Value.kForward);
  }

  public void closeClaw(){
    //close piston
    clawSolenoid.set(Value.kReverse);
  }

  public void runClawtake(){
    //run the rotoclawtake/grab a game piece
    clawTakeMotor.set(1);
  }
  public void reverseClawtake(){
    //spit out a game piece/outtake rotoclawtake
    clawTakeMotor.set(-1);
  }

  public void rotoClaw(GamePieceOrientation gamePieceOrientation){
    //twist the rotoclawtake
    rotoMotorPID.setReference(gamePieceOrientation.getRotOrientForRoto(), ControlType.kPosition);
  }

  public void actuateClaw(GamePieceOrientation gamePieceOrientation){

    if (gamePieceOrientation.getGamePieceType() == GamePieceType.CONE){
      clawMotorPID.setReference(0, ControlType.kPosition);
    } else { // else is cube
      clawMotorPID.setReference(4, ControlType.kPosition);
    }

  }
  

}
