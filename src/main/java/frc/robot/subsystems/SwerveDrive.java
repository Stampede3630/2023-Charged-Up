// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.QuadFalconSwerveDrive;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.util.LimelightPose2d;
import frc.robot.util.SimGyroSensorModel;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import java.sql.Driver;
import java.util.function.DoubleSupplier;


public class SwerveDrive extends SubsystemBase implements Loggable {
  QuadFalconSwerveDrive m_driveTrain;
  Pose2d prevRobotPose = new Pose2d();
  Pose2d robotPose = new Pose2d();
  Pose2d visionPoseFront = new Pose2d();
  double deltaTime = 0;
  double prevTime = 0;
  AHRS gyro;
  SimGyroSensorModel simNavx; 
  SwerveDrivePoseEstimator m_odometry; 
  double holdHeadingAngle = 0;
  boolean holdHeadingEnabled = false;
  ProfiledPIDController rotationController, rollRotationController, pitchRotationController;
  double rotationControllerOutput;

  boolean aprilTagDetectedFront = false;
  boolean aprilTagDetectedBack = false;

  boolean balanced=false;

  @Log
  public Field2d m_field;
  public double[] akitPose = {0,0,0};

  public SwerveDrive() {
    rollRotationController =new ProfiledPIDController(5.4/67, 0, 0, 
      new TrapezoidProfile.Constraints(.7,.7));
    pitchRotationController = new ProfiledPIDController(5.4/67, 0, 0, 
      new TrapezoidProfile.Constraints(.7,.7));
    rotationController = new ProfiledPIDController(
      Preferences.getDouble("pKPRotationController", SwerveConstants.P_ROTATION_CONTROLLER),
      Preferences.getDouble("pIPRotationController", SwerveConstants.I_ROTATION_CONTROLLER),
      Preferences.getDouble("pKDRotationController", SwerveConstants.D_ROTATION_CONTROLLER),
      new TrapezoidProfile.Constraints(Units.radiansToDegrees(SwerveConstants.MAX_SPEED_RADIANSperSECOND), 5*Units.radiansToDegrees(SwerveConstants.MAX_SPEED_RADIANSperSECOND)));
    rotationController.enableContinuousInput(-180.0, 180.0);
    rotationController.setTolerance(4.0);
    //NAVX gyro and sim setup
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset(); 
    gyro.setAngleAdjustment(0); // 135 TODO: CHANGE THIS FOR FINAL BOT!!!!!!!!!!!!!!!
    simNavx = new SimGyroSensorModel();

    //SwerveDrive Setup
    m_driveTrain = new QuadFalconSwerveDrive();
    // m_driveTrain.checkAndSeedALLSwerveAngles();
    
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
    prevRobotPose = m_odometry.getEstimatedPosition();
    if(RobotBase.isSimulation()) {
      for(SwerveModule module : m_driveTrain.SwerveModuleList) {
        module.simModule.simulationPeriodic(deltaTime);
      }    

    }

    deltaTime = Timer.getFPGATimestamp() - prevTime;
    prevTime = Timer.getFPGATimestamp();


    if(RobotBase.isSimulation()) {
      simNavx.update(robotPose, prevRobotPose, deltaTime);
    }

    aprilTagDetectedFront = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tv").getDouble(0) == 1;
    aprilTagDetectedBack = NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("tv").getDouble(0) == 1;
    // System.out.println(aprilTagDetected);

    robotPose = updateOdometry();

    LimelightPose2d frontPose = limelightBotPoseFront();
    if (aprilTagDetectedFront && frontPose.getLatency() < 1.5) {
      m_odometry.addVisionMeasurement(frontPose, Timer.getFPGATimestamp() - frontPose.getLatency());
    }

    LimelightPose2d backPose = limelightBotPoseBack();
    if(aprilTagDetectedBack && backPose.getLatency() < 1.5) {
      m_odometry.addVisionMeasurement(backPose, Timer.getFPGATimestamp() - backPose.getLatency());
    }

    drawRobotOnField(m_field);
    updateRotationController();
  }


  public CommandBase joystickDriveCommand(DoubleSupplier _x, DoubleSupplier _y, DoubleSupplier _rot){
    return Commands.run(
      () -> {
        double x = -(Math.abs(_x.getAsDouble()) < .1 ? 0 : _x.getAsDouble());
        double y = -(Math.abs(_y.getAsDouble()) < .1 ? 0 : _y.getAsDouble());
        double rot = -Math.pow((Math.abs(_rot.getAsDouble()) < .1 ? 0 : _rot.getAsDouble()), 3);
        double joystickDriveGovernor = Preferences.getDouble("pDriveGovernor", Constants.DriverConstants.DRIVE_GOVERNOR);
        if (Preferences.getBoolean("pAccelInputs", Constants.DriverConstants.ACCELERATED_INPUTS)) {

        } else {
          x = Math.signum(x) * Math.sqrt(Math.abs(x));
          y = Math.signum(y) * Math.sqrt(Math.abs(y));
          rot = Math.signum(rot) * Math.sqrt(Math.abs(rot));
        }

        if (Math.abs(x) < 0.00001 || Math.abs(y) < 0.00001 || Math.abs(rot) < 0.00001) {
          SwerveModuleState[] noMoveStates = new SwerveModuleState[4];
          SwerveModulePosition[] currentPositions = m_driveTrain.getModulePositions();
          for (int i = 0; i < currentPositions.length; i++) {
            noMoveStates[i] = new SwerveModuleState(0, currentPositions[i].angle);
          }
          m_driveTrain.setModuleSpeeds(noMoveStates);
        } else {
          setDriveSpeeds(
            new Translation2d(
              convertToMetersPerSecond(x)*joystickDriveGovernor,
              convertToMetersPerSecond(y)*joystickDriveGovernor), 
            convertToRadiansPerSecond(rot)* joystickDriveGovernor, 
            Preferences.getBoolean("pFieldRelative", Constants.DriverConstants.FIELD_RELATIVE));
        }
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

  public boolean getAtGoal(){
    return rotationController.atGoal();
  }
 
  /**
   * @return a Rotation2d populated by the gyro readings 
   * or estimated by encoder wheels (if gyro is disconnected)
   */
  public Rotation2d getRobotAngle(){
    // if(RobotBase.isSimulation() || !gyro.isConnected()) {
      // return prevRobotPose.getRotation().rotateBy(new Rotation2d(m_driveTrain.m_kinematics.toChassisSpeeds(m_driveTrain.getModuleStates()).omegaRadiansPerSecond *deltaTime));   
    // }
    return gyro.getRotation2d();
  }

  public Pose2d getPose() {
    return robotPose;
  }

  @Log
  public double odometryDegrees() {
    return getPose().getRotation().getDegrees();
  }

  public double getRobotAngleDegrees() {
    return getRobotAngle().getDegrees();
  }

  /**
   * @param xySpeedsMetersPerSec (X is Positive forward, Y is Positive Right)
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
   */
  public void drawRobotOnField(Field2d field) {
    Pose2d robotPose = m_odometry.getEstimatedPosition();
    if (DriverStation.getAlliance() == Alliance.Red) {
      robotPose = new Pose2d(new Translation2d(16.541748984 - robotPose.getX(), 8.01367968 - robotPose.getY()), robotPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }
    field.setRobotPose(robotPose);
    
    field.getObject("frontLeft").setPose(
      robotPose.transformBy(new Transform2d(m_driveTrain.FrontLeftSwerveModule.moduleXYTranslation, m_driveTrain.FrontLeftSwerveModule.getPosition().angle)));
    field.getObject("frontRight").setPose(
      robotPose.transformBy(new Transform2d(m_driveTrain.FrontRightSwerveModule.moduleXYTranslation, m_driveTrain.FrontRightSwerveModule.getPosition().angle)));
    field.getObject("backLeft").setPose(
      robotPose.transformBy(new Transform2d(m_driveTrain.BackLeftSwerveModule.moduleXYTranslation, m_driveTrain.BackLeftSwerveModule.getPosition().angle)));
    field.getObject("backRight").setPose(
      robotPose.transformBy(new Transform2d(m_driveTrain.BackRightSwerveModule.moduleXYTranslation, m_driveTrain.BackRightSwerveModule.getPosition().angle)));
    akitPose[0] = robotPose.getX();
    akitPose[1] = robotPose.getY();
    akitPose[2] = robotPose.getRotation().getRadians();
    SmartDashboard.putNumberArray("akitPose", akitPose);
  }

  public void setToBrake(){
    m_driveTrain.setToBrake();
  }

  public void setToCoast(){
    m_driveTrain.setToCoast();
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

  // public Command setToCoast(){
  //   return new RunCommand(()->m_driveTrain.setToCoast());
  // }

  private double convertToMetersPerSecond(double _input){
    return _input*SwerveConstants.MAX_SPEED_METERSperSECOND;
  }

  private double convertToRadiansPerSecond(double _input){
    return _input*SwerveConstants.MAX_SPEED_RADIANSperSECOND;
  }
  
  public LimelightPose2d limelightBotPoseFront(){

    String allianceColorBotPose = DriverStation.getAlliance() == Alliance.Red ? "botpose_wpired" : "botpose_wpiblue";

    double[] myArray = {0, 0, 0, 0, 0, 0, 0};
    
    myArray = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry(allianceColorBotPose).getDoubleArray(myArray);

    double x = 0;
    double y = 0;
    double rot = 0;
    if (myArray.length > 0){
      x = myArray[0];
      y = myArray[1];
      rot = myArray[5];
    }

    return new LimelightPose2d(x, y, Rotation2d.fromDegrees(rot), myArray[6]/1000.0);

  }

  public LimelightPose2d limelightBotPoseBack(){

    double[] myArray = {0, 0, 0, 0, 0, 0, 0, 6};
    String allianceColorBotPose = DriverStation.getAlliance() == Alliance.Red ? "botpose_wpired" : "botpose_wpiblue";

      myArray = NetworkTableInstance.getDefault().getTable("limelight-back").getEntry(allianceColorBotPose).getDoubleArray(myArray);
    

    double x = 0;
    double y = 0;
    double rot = 0;
    if (myArray.length > 0){
      x = myArray[0];
      y = myArray[1];
      rot = myArray[5];
    }

    return new LimelightPose2d(x, y, Rotation2d.fromDegrees(rot), myArray[6]/1000.0);

  }

  @Config(defaultValueBoolean = false)
  public void zeroOdometry(boolean input){
    if(input){
      m_odometry.resetPosition(gyro.getRotation2d(), m_driveTrain.getModulePositions(), new Pose2d());
      gyro.reset();
    }
  }

  @Log // using @Log to run something periodically
  public double  trapRollAutoBalance(){
    return rollRotationController.calculate(gyro.getRoll(), new State(0,0));
  }

  @Log
  public double  trapPitchAutoBalance(){
    return -pitchRotationController.calculate(gyro.getPitch(), new State(0,0));
  }

  public void autoBalance(){
    if((Math.abs(gyro.getPitch()) < 2) && (Math.abs(gyro.getRoll()) < 2)){
      setDriveSpeeds(new Translation2d(0, 0), 0, false);
      m_driveTrain.activateDefensiveStop(getPose().getRotation());
      balanced = true;
    } else {
      setDriveSpeeds(new Translation2d(trapPitchAutoBalance(),trapRollAutoBalance()), 0, false);
      balanced= false;
    }
    if (DriverStation.getMatchTime() < 0.2) { // activate defensive if running out of time
      m_driveTrain.activateDefensiveStop(getPose().getRotation());
    }
  }

  public void activateDefensiveStop() {
    m_driveTrain.activateDefensiveStop(getPose().getRotation());
  }

  public Command autoBalanceCommand(){
    return Commands.run(this::autoBalance).until(() -> balanced).andThen(Commands.run(this::activateDefensiveStop));
  }

  public boolean isBalanced() {
      return balanced;
  }

  @Log
  public double getRoll() {
    return gyro.getRoll();
  }

  @Log
  public double getPitch() {
    return gyro.getPitch();
  }
  
  /* 
   * if pitch is negative, go forward +y
   * if pitch is positive, go backward -y
   * if roll is positive, go right +x
   * if roll is negative, go left -x
   */
  
}

  

