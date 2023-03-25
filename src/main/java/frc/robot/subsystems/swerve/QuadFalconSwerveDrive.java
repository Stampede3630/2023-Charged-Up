package frc.robot.subsystems.swerve;

import java.util.List;

import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.util.SwerveKinematics2;
import frc.robot.util.SwerveModuleState2;

public class QuadFalconSwerveDrive {
  public NeutralModeValue NeutralMode = NeutralModeValue.Brake;
  // public TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

  public SwerveModule FrontLeftSwerveModule = new SwerveModule(
      new SwerveModule.DriveMotor(SwerveConstants.FLDriveID, SwerveConstants.FLInvertType,
          SwerveConstants.FLDriveGains),
      new SwerveModule.SteeringMotor(SwerveConstants.FLSteerID, SwerveConstants.FLSteerGains),
      new SwerveModule.SteeringSensor(SwerveConstants.FLSensorID, SwerveConstants.FLSensorOffset),
      new Translation2d(SwerveConstants.WHEEL_BASE_METERS / 2, SwerveConstants.TRACK_WIDE / 2));

  public SwerveModule BackLeftSwerveModule = new SwerveModule(
      new SwerveModule.DriveMotor(SwerveConstants.BLDriveID, SwerveConstants.BLInvertType,
          SwerveConstants.BLDriveGains),
      new SwerveModule.SteeringMotor(SwerveConstants.BLSteerID, SwerveConstants.BLSteerGains),
      new SwerveModule.SteeringSensor(SwerveConstants.BLSensorID, SwerveConstants.BLSensorOffset),
      new Translation2d(-SwerveConstants.WHEEL_BASE_METERS / 2, SwerveConstants.TRACK_WIDE / 2));

  public SwerveModule BackRightSwerveModule = new SwerveModule(
      new SwerveModule.DriveMotor(SwerveConstants.BRDriveID, SwerveConstants.BRInvertType,
          SwerveConstants.BRDriveGains),
      new SwerveModule.SteeringMotor(SwerveConstants.BRSteerID, SwerveConstants.BRSteerGains),
      new SwerveModule.SteeringSensor(SwerveConstants.BRSensorID, SwerveConstants.BRSensorOffset),
      new Translation2d(-SwerveConstants.WHEEL_BASE_METERS / 2, -SwerveConstants.TRACK_WIDE / 2));

  public SwerveModule FrontRightSwerveModule = new SwerveModule(
      new SwerveModule.DriveMotor(SwerveConstants.FRDriveID, SwerveConstants.FRInvertType,
          SwerveConstants.FRDriveGains),
      new SwerveModule.SteeringMotor(SwerveConstants.FRSteerID, SwerveConstants.FRSteerGains),
      new SwerveModule.SteeringSensor(SwerveConstants.FRSensorID, SwerveConstants.FRSensorOffset),
      new Translation2d(SwerveConstants.WHEEL_BASE_METERS / 2, -SwerveConstants.TRACK_WIDE / 2));

  public SwerveKinematics2 m_kinematics = new SwerveKinematics2(
      FrontLeftSwerveModule.moduleXYTranslation,
      FrontRightSwerveModule.moduleXYTranslation,
      BackLeftSwerveModule.moduleXYTranslation,
      BackRightSwerveModule.moduleXYTranslation);

  public final List<SwerveModule> SwerveModuleList = List.of(
      FrontLeftSwerveModule,
      FrontRightSwerveModule,
      BackLeftSwerveModule,
      BackRightSwerveModule);


  public String getSteerMethodStrings() {
    return FrontLeftSwerveModule.steerMode +
        BackLeftSwerveModule.steerMode +
        BackRightSwerveModule.steerMode +
        FrontRightSwerveModule.steerMode;
  }

  public void activateDefensiveStop(Rotation2d robotAngle) {
    FrontLeftSwerveModule.setDesiredState(new SwerveModuleState2(0, robotAngle.rotateBy(Rotation2d.fromDegrees(90)),0));
    FrontRightSwerveModule.setDesiredState(new SwerveModuleState2(0, robotAngle.rotateBy(Rotation2d.fromDegrees(90)),0));
    BackLeftSwerveModule.setDesiredState(new SwerveModuleState2(0, robotAngle.rotateBy(Rotation2d.fromDegrees(90)),0));
    BackRightSwerveModule.setDesiredState(new SwerveModuleState2(0, robotAngle.rotateBy(Rotation2d.fromDegrees(90)),0)); 
  }

  public void setModuleSpeeds(SwerveModuleState2[] _swerveModuleStates) {
    FrontLeftSwerveModule.setDesiredState(_swerveModuleStates[0]);
    BackLeftSwerveModule.setDesiredState(_swerveModuleStates[2]);
    BackRightSwerveModule.setDesiredState(_swerveModuleStates[3]);
    FrontRightSwerveModule.setDesiredState(_swerveModuleStates[1]);
  }

  /**
   * MUST BE ADDED TO PERIODIC (NOT INIT METHODS)
   * sets all the talons (steer and drive motors) to coast.
   * This allows for easy moving of the robot
   */
  public void setToCoast() {
    if (NeutralMode == NeutralModeValue.Brake &&
        Math.abs(FrontLeftSwerveModule.driveMotor.getVelocity().getValue()) < 100 &&
        Math.abs(BackLeftSwerveModule.driveMotor.getVelocity().getValue()) < 100 &&
        Math.abs(FrontRightSwerveModule.driveMotor.getVelocity().getValue()) < 100 &&
        Math.abs(BackRightSwerveModule.driveMotor.getVelocity().getValue()) < 100) {
      FrontRightSwerveModule.setModuleToCoast();
      BackRightSwerveModule.setModuleToCoast();
      FrontLeftSwerveModule.setModuleToCoast();
      BackLeftSwerveModule.setModuleToCoast();
      NeutralMode = NeutralModeValue.Coast;
    }
  }

  public void setToBrake() {
    FrontRightSwerveModule.setModuleToBrake();
    BackRightSwerveModule.setModuleToBrake();
    FrontLeftSwerveModule.setModuleToBrake();
    BackLeftSwerveModule.setModuleToBrake();
    NeutralMode = NeutralModeValue.Brake;
  }


  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        FrontLeftSwerveModule.getPosition(),
        FrontRightSwerveModule.getPosition(),
        BackLeftSwerveModule.getPosition(),
        BackRightSwerveModule.getPosition()
    };
  }

}
