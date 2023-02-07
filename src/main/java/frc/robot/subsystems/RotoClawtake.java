// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotoClawtake extends SubsystemBase {
  /** Creates a new Claw. */
  public RotoClawtake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void openClaw(){
    //open piston
  }

  public void closeClaw(){
    //close piston
  }

  public void runClawtake(){
    //run the rotoclawtake/grab a game piece
  }
  public void reverseClawtake(){
    //spit out a game piece/outtake rotoclawtake
  }

  public void rotoClaw(){
    //twist the rotoclawtake
  }
}
