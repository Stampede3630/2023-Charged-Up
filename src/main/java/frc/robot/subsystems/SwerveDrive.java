// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.QuadFalconSwerveDrive;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.util.SimGyroSensorModel;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.SPI;


public class SwerveDrive extends SubsystemBase implements Loggable {
  QuadFalconSwerveDrive m_driveTrain;
  Pose2d prevRobotPose = new Pose2d();
  Pose2d robotPose = new Pose2d();
  Pose2d visionPose = new Pose2d();
  double deltaTime = 0;
  double prevTime = 0;
  AHRS gyro;
  SimGyroSensorModel simNavx; 
  SwerveDrivePoseEstimator m_odometry; 
  @Log
  double holdHeadingAngle = 0;
  @Log
  boolean holdHeadingEnabled = false;
  ProfiledPIDController rotationController;
  double rotationControllerOutput;

  double aprilTagDetected = 0;

  @Log
  public Field2d m_field;

  public SwerveDrive() {
    rotationController = new ProfiledPIDController(
      Preferences.getDouble("pKPRotationController", SwerveConstants.kPRotationController),
      Preferences.getDouble("pIPRotationController", SwerveConstants.kIRotationController),
      Preferences.getDouble("pKDRotationController", SwerveConstants.kDRotationController),
      new TrapezoidProfile.Constraints(Units.radiansToDegrees(SwerveConstants.MAX_SPEED_RADIANSperSECOND), 5*Units.radiansToDegrees(SwerveConstants.MAX_SPEED_RADIANSperSECOND)));
    rotationController.enableContinuousInput(-180.0, 180.0);
    rotationController.setTolerance(4.0);
    //NAVX gyro and sim setup
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset(); 
    simNavx = new SimGyroSensorModel();

    //SwerveDrive Setup
    m_driveTrain = new QuadFalconSwerveDrive();
    m_driveTrain.checkAndSetSwerveCANStatus();
    m_driveTrain.checkAndSeedALLSwerveAngles();
    
    //helps visualize robot on virtual field
    m_field = new Field2d();
    
    //Uses 2023 new Pose Estimators which is a drop-in replacement(mostly) for odometry
    m_odometry =     
    new SwerveDrivePoseEstimator(
      m_driveTrain.m_kinematics, 
      getRobotAngle(),
      m_driveTrain.getModulePositions(), 
      robotPose, 
      VecBuilder.fill(0.001, 0.001, Units.degreesToRadians(.1)), 
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(30)));
    }

  @Override
  public void periodic() {
    m_driveTrain.checkAndSeedALLSwerveAngles();
    prevRobotPose = m_odometry.getEstimatedPosition();
    if(RobotBase.isSimulation()) {
      for(SwerveModule module : m_driveTrain.SwerveModuleList) {
        module.simModule.simulationPeriodic(deltaTime);
      }    

    } 

    // System.out.println(visionPose);
    // System.out.println(limelightLatency());
    // System.out.println(m_odometry);

    deltaTime = Timer.getFPGATimestamp() - prevTime;
    prevTime = Timer.getFPGATimestamp();

    if(RobotBase.isSimulation()) {
      simNavx.update(robotPose, prevRobotPose, deltaTime);
    }

    aprilTagDetected = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(aprilTagDetected);

    // System.out.println(aprilTagDetected);

    robotPose = updateOdometry();
    visionPose = limelightBotPose();

    // if (aprilTagDetected == 1) {
    //   m_odometry.addVisionMeasurement(visionPose, Timer.getFPGATimestamp() - limelightLatency());
    // } else {
    //   updateOdometry();
    // }


    m_driveTrain.checkAndSetSwerveCANStatus();
    drawRobotOnField(m_field);
    updateRotationController();
  }


  /**
   * @param _x
   * @param _y
   * @param _rot
   * @param driveGovernor
   * @param fieldRelative
   * @param acceleratedInputs
   */
  public CommandBase joystickDriveCommand(DoubleSupplier _x, DoubleSupplier _y, DoubleSupplier _rot){
    return Commands.run(
      () -> {
        double x = -_x.getAsDouble();
        double y = -_y.getAsDouble();
        double rot = -_rot.getAsDouble();
        double joystickDriveGovernor = Preferences.getDouble("pDriveGovernor", Constants.driveGovernor);
        
        if (Preferences.getBoolean("pAccelInputs", Constants.acceleratedInputs)) {

        } else {
          x = Math.signum(x) * Math.sqrt(Math.abs(x));
          y = Math.signum(y) * Math.sqrt(Math.abs(y));
          rot = Math.signum(rot) * Math.sqrt(Math.abs(rot));
        }
        setDriveSpeeds(
          new Translation2d(
            convertToMetersPerSecond(x)*joystickDriveGovernor,
            convertToMetersPerSecond(y)*joystickDriveGovernor), 
          convertToRadiansPerSecond(rot)* joystickDriveGovernor, 
          Preferences.getBoolean("pFieldRelative", Constants.fieldRelative));
        }, this);
  }

  public Command holdHeadingCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    return joystickDriveCommand(
                xSpeed,
                ySpeed,
                ()->rotationControllerOutput
            );
  }

  public void updateRotationController(){
    if(holdHeadingEnabled){
      rotationControllerOutput = rotationController.calculate(
                  Math.IEEEremainder(getRobotAngleDegrees(), 360),
                  new State(holdHeadingAngle, 0.0))/-Units.radiansToDegrees(SwerveConstants.MAX_SPEED_RADIANSperSECOND);
    }
  }

  @Log
  public boolean getAtGoal(){
    return rotationController.atGoal();
  }
 
  /**
   * @return a Rotation2d populated by the gyro readings 
   * or estimated by encoder wheels (if gyro is disconnected)
   */
  public Rotation2d getRobotAngle(){
    if(RobotBase.isSimulation() || !gyro.isConnected()) {
      return prevRobotPose.getRotation().rotateBy(new Rotation2d(m_driveTrain.m_kinematics.toChassisSpeeds(m_driveTrain.getModuleStates()).omegaRadiansPerSecond *deltaTime));   
    } else return gyro.getRotation2d();
  }

  @Log
  public double getRobotAngleDegrees() {
    return getRobotAngle().getDegrees();
  }

  /**
   * @param xySpeedsMetersPerSec (X is Positive forward, Y is Positive Right)
   * @param rRadiansPerSecond 
   * @param fieldRelative (SUGGESTION: Telop use field centric, AUTO use robot centric)
   */
  public void setDriveSpeeds(Translation2d xySpeedsMetersPerSec, double rRadiansPerSecond, boolean fieldRelative){
    SwerveModuleState[] swerveModuleStates =
      m_driveTrain.m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            xySpeedsMetersPerSec.getX(), 
                            xySpeedsMetersPerSec.getY(), 
                            rRadiansPerSecond, 
                            getRobotAngle()
                        )
                        : new ChassisSpeeds(
                            xySpeedsMetersPerSec.getX(), 
                            xySpeedsMetersPerSec.getY(), 
                            rRadiansPerSecond)
                        );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED_METERSperSECOND);
    m_driveTrain.setModuleSpeeds(swerveModuleStates);
  }

  /** 
   * Update the SwerveDrivePoseEstimator
  */
  public Pose2d updateOdometry(){
    return m_odometry.update(getRobotAngle(),
      m_driveTrain.getModulePositions());
  }

  public Pose2d getOdometryPose(){
    return m_odometry.getEstimatedPosition();
  }

  public void setHoldHeadingFlag(Boolean input){
    holdHeadingEnabled = input;
  }
  @Log
  public boolean getHoldHeadingFlag(){
    return holdHeadingEnabled;
  }

  public void setHoldHeadingAngle(double input) {
    holdHeadingAngle = input;
  }

  public void resetOdometry(Pose2d newPose){
    m_odometry.resetPosition(newPose.getRotation(), m_driveTrain.getModulePositions(), newPose);
  }

  public SwerveDriveKinematics getKinematics() {
    return m_driveTrain.m_kinematics;
  }
  
  public void setAutoModuleStates (SwerveModuleState[] states){
    m_driveTrain.setModuleSpeeds(states);
  }

  /**
   * Draw a pose that is based on the robot pose, but shifted by the translation of the module relative to robot center,
   * then rotated around its own center by the angle of the module.
   * @param field
   */
  public void drawRobotOnField(Field2d field) {
    field.setRobotPose(m_odometry.getEstimatedPosition());

    field.getObject("frontLeft").setPose(
      m_odometry.getEstimatedPosition().transformBy(new Transform2d(m_driveTrain.FrontLeftSwerveModule.moduleXYTranslation, m_driveTrain.FrontLeftSwerveModule.getPosition().angle)));
    field.getObject("frontRight").setPose(
      m_odometry.getEstimatedPosition().transformBy(new Transform2d(m_driveTrain.FrontRightSwerveModule.moduleXYTranslation, m_driveTrain.FrontRightSwerveModule.getPosition().angle)));
    field.getObject("backLeft").setPose(
      m_odometry.getEstimatedPosition().transformBy(new Transform2d(m_driveTrain.BackLeftSwerveModule.moduleXYTranslation, m_driveTrain.BackLeftSwerveModule.getPosition().angle)));
    field.getObject("backRight").setPose(
      m_odometry.getEstimatedPosition().transformBy(new Transform2d(m_driveTrain.BackRightSwerveModule.moduleXYTranslation, m_driveTrain.BackRightSwerveModule.getPosition().angle)));
  }

  public Command setToBrake(){
    return new InstantCommand(()->m_driveTrain.setToBrake());
  }

  @Log
  public double xMeters(){
    return m_odometry.getEstimatedPosition().getX();
  }

  @Log
  public double yMeters(){
    return m_odometry.getEstimatedPosition().getY();
  }

  /**
   * METHOD WILL NOT WORK UNLESS ADDED TO PERIODIC
   */

  public Command setToCoast(){
    return new RunCommand(()->m_driveTrain.setToCoast());
  }

  private double convertToMetersPerSecond(double _input){
    return _input*SwerveConstants.MAX_SPEED_METERSperSECOND;
  }

  private double convertToRadiansPerSecond(double _input){
    return _input*SwerveConstants.MAX_SPEED_RADIANSperSECOND;
  }

  
  public Command switchToRemoteSteerCommand(){
    return new InstantCommand(() -> m_driveTrain.switchToRemoteSteering(),this);
  }

  public Command switchToIntegratedSteerCommand(){
    return new InstantCommand(() -> m_driveTrain.switchToIntegratedSteer(),this);
  }


  
  public Pose2d limelightBotPose(){

    double myArray[] = {0, 0, 0, 0, 0, 0};
    
    myArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(myArray);

    double x = 8.2425 - myArray[0];
    double y = 4.0515 + myArray[1];
    double rot = myArray[5];

    return new Pose2d(x, y, Rotation2d.fromDegrees(rot));

  }

  public double limelightLatency(){
    double vLatency = 0;
    vLatency = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(vLatency);
   
    return (vLatency * 0.001) + (11 * 0.001);
  }

}
