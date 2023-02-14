// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class RotoClawtake extends SubsystemBase implements Loggable {

  public double rotoMeasure;

  public Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
  public DoubleSolenoid clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
  public CANSparkMax clawTakeMotor = new CANSparkMax(14, MotorType.kBrushless);
  public CANSparkMax rotoMotor = new CANSparkMax(13, MotorType.kBrushless);
  public CANSparkMax clawMotor = new CANSparkMax(19, MotorType.kBrushless);

  public RelativeEncoder rotoRelativeEncoder = rotoMotor.getEncoder();
  public RelativeEncoder clawRelativeEncoder = clawMotor.getEncoder();

  // public AbsoluteEncoder rotoAbsolute = rotoMotor.getAbsoluteEncoder(Type.kDutyCycle);

  public DigitalInput clawSwitch = new DigitalInput(0);

  public SparkMaxPIDController rotoMotorPID = rotoMotor.getPIDController();
  public SparkMaxPIDController clawMotorPID = clawMotor.getPIDController();

  /** Creates a new Claw. */
  public RotoClawtake() {

    rotoRelativeEncoder.setPositionConversionFactor((2 * Math.PI) / 60.0);
    clawRelativeEncoder.setPositionConversionFactor((2 * Math.PI) / 22.8571428571);

    clawMotorPID.setP(Preferences.getDouble("ClawKP", 6.0 / Math.PI));
    clawMotorPID.setI(Preferences.getDouble("ClawKI", 0.0));
    clawMotorPID.setD(Preferences.getDouble("ClawPD", 0.0));

    rotoMotorPID.setP(Preferences.getDouble("ExtensionKP", 6.0 / Math.PI));
    rotoMotorPID.setI(Preferences.getDouble("ExtensionKI", 0.0));
    rotoMotorPID.setD(Preferences.getDouble("ExtensionPD", 0.0));

    rotoMotor.burnFlash();
    clawTakeMotor.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Preferences.getBoolean("Wanna PID", false)) {
      rotoMotorPID.setP(Preferences.getDouble("ExtensionKP", 1.0 / 6.0));
      rotoMotorPID.setI(Preferences.getDouble("ExtensionKI", 0.0));
      rotoMotorPID.setD(Preferences.getDouble("ExtensionPD", 0.0));
    }
  }

  public void stopClawTake() {
    clawTakeMotor.set(0);
    rotoMotor.set(0);
    clawMotor.set(0);
  }

  public void openClaw() {
    // open piston
    // clawSolenoid.set(Value.kForward);
    clawMotorPID.setReference(4, ControlType.kPosition);
  }

  public void closeClaw() {
    // close piston
    // clawSolenoid.set(Value.kReverse);
    clawMotorPID.setReference(0, ControlType.kPosition);

  }

  public void runClawtake() {
    // run the rotoclawtake/grab a game piece
    clawTakeMotor.set(-1);
  }

  public void reverseClawtake() {
    // spit out a game piece/outtake rotoclawtake
    clawTakeMotor.set(1);
  }

  public void rotoClaw() { // GamePieceOrientation gamePieceOrientation add this parameter later
    // twist the rotoclawtake
    // rotoMotorPID.setReference(gamePieceOrientation.getRotOrientForRoto(),
    // ControlType.kPosition);
    rotoMotor.set(0.7);
  }

  public void rotoClawReverse() { // delete this method after testing
    rotoMotor.set(-0.7);

  }

  public void actuateClaw() { // add parameter GamePieceOrientation gamePieceOrientation?

    clawMotor.set(0.2);

    // if (gamePieceOrientation.getGamePieceType().equals("cone")){
    // clawMotorPID.setReference(0, ControlType.kPosition);
    // } else {
    // clawMotorPID.setReference(4, ControlType.kPosition);
    // }

  }

  public void actuateClawReverse() {
    clawMotor.set(-0.2); // delete this method after testing
  }

}
