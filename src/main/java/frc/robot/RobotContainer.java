// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.RotoClawtake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TheCannon;
import frc.robot.subsystems.swerve.SwerveConstants;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Config.PIDCommand;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  /*Controller setup.  For simulations google: x360CE */
  private final CommandXboxController xBox = new CommandXboxController(0);

  private boolean isIntegratedSteering = true;
  SwerveAutoBuilder autoBuilder;
  ArrayList<PathPlannerTrajectory> autoPathGroup, leftPathGroup, rightPathGroup;

  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.


// This is just an example event map. It would be better to have a constant, global event map

  
@Log
private final SwerveDrive s_SwerveDrive = new SwerveDrive();
  // The robot's subsystems and commands are defined here...
private final RotoClawtake s_Claw = new RotoClawtake();

private final TheCannon s_Cannon = new TheCannon();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /**
     * Preferences are cool.  they store the values in the roborio flash memory so they don't necessarily get reset to default.  
     */
    Preferences.initBoolean("pFieldRelative", Constants.fieldRelative);
    Preferences.initBoolean("pAccelInputs", Constants.acceleratedInputs);
    Preferences.initDouble("pDriveGovernor", Constants.driveGovernor);
    Preferences.initBoolean("pOptimizeSteering", SwerveConstants.OPTIMIZESTEERING);
    Preferences.initDouble("pKPRotationController", SwerveConstants.kPRotationController);
    Preferences.initDouble("pKIRotationController", SwerveConstants.kDRotationController);
    Preferences.initDouble("pKDRotationController", SwerveConstants.kIRotationController);
    Preferences.initBoolean("pIntSteering", true);

    autoPathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("swerveTest", new PathConstraints(2, 2));

    Shuffleboard.getTab("nodeSelector")
      .add("node", "");


    SendableChooser<GamePieceOrientation> gamePieceOrientationChooser = new SendableChooser<>();
    for (GamePieceOrientation orientation : GamePieceOrientation.values()) {
      gamePieceOrientationChooser.addOption(orientation.getFriendlyName(), orientation);
    }

    SendableChooser<NodeDriverStation> nodeDriverStation = new SendableChooser<>();
      for (NodeDriverStation ds : NodeDriverStation.values()) {
        nodeDriverStation.addOption(ds.dsFriendlyName, ds);
      }

      Shuffleboard.getTab("nodeSelector")
        .add("orientation", gamePieceOrientationChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);

      Shuffleboard.getTab("nodeSelector")
        .add("Node Driver Station", nodeDriverStation)
        .withWidget(BuiltInWidgets.kComboBoxChooser);

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("1stBallPickup", new WaitCommand(2));
    eventMap.put("2ndBallPickup", new WaitCommand(2));
    eventMap.put("3rdBallPickup", new WaitCommand(2));

    autoBuilder = new SwerveAutoBuilder(
      s_SwerveDrive::getOdometryPose, // Pose2d supplier
      s_SwerveDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      s_SwerveDrive.getKinematics(), // SwerveDriveKinematics
      new PIDConstants(5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      s_SwerveDrive::setAutoModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      s_SwerveDrive // The drive subsystem. Used to properly set the requirements of path following commands
    );


    s_SwerveDrive.setDefaultCommand(
        s_SwerveDrive.joystickDriveCommand(
          xBox::getLeftY,
          xBox::getLeftX,
          xBox::getRightX).withName("DefaultDrive"));

    // s_Cannon.setDefaultCommand(
    //   new InstantCommand(s_Cannon::stowMode, s_Cannon)
    // );

    // s_Claw.setDefaultCommand(
    //   new FunctionalCommand(() -> System.out.println("default"), () -> {}, interrupted -> {}, () -> true, s_Claw)
    // );


    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    //autoPathGroup = ;//PathPlanner.loadPathGroup("Test5Ball", new PathConstraints(4, 3));
    //leftPathGroup = ;//PathPlanner.loadPathGroup("LeftPath1WP", new PathConstraints(4, 3));
    //rightPathGroup = ;//PathPlanner.loadPathGroup("RightPath1WP", new PathConstraints(4, 3));
    // Configure the button bindings
    configureButtonBindings();
    Logger.configureLoggingAndConfig(this, false);
  }
  @Config(defaultValueNumeric = 0)
  private void setCannonPosition(double value){
    s_Cannon.setReference(value);
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(()->Preferences.getBoolean("pIntSteering", true))
      .onFalse(s_SwerveDrive.switchToRemoteSteerCommand().ignoringDisable(true))
      .onTrue(s_SwerveDrive.switchToIntegratedSteerCommand().ignoringDisable(true));
    
    /**
     * Trigger Coast/Brake modes when DS is Disabled/Enabled.
     * Trigger runs WHILETRUE for coast mode.  Coast Mode method
     * is written to wait for slow speeds before setting to coast
     */
    new Trigger(DriverStation::isDisabled)
      .whileTrue(s_SwerveDrive.setToCoast()
      .ignoringDisable(true)
      .withName("SetToCoast"))
      .onFalse(s_SwerveDrive.setToBrake()
      .withName("setToBrake"));


    /**
     * next two triggers are to "toggle" rotation HOLD mode and set a heading
     * */  

    new Trigger(()->Math.abs(xBox.getRightX()) < .1)
      .and(s_SwerveDrive::getHoldHeadingFlag)
      .and(new Trigger(s_SwerveDrive::getAtGoal).negate())
        .whileTrue(
          s_SwerveDrive.holdHeadingCommand(
            xBox::getLeftY,
            xBox::getLeftX)
          .withName("holdHeading")
          )
        .whileFalse(
          s_SwerveDrive.joystickDriveCommand(
            xBox::getLeftY,
            xBox::getLeftX,
            xBox::getRightX)
          .withName("StandardOperatorDrive")
        );

    xBox.b()
        .onTrue(new InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(false)));

    // xBox.povCenter().negate().onTrue(
    //     new SequentialCommandGroup(
    //       new InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(true)),
    //       new InstantCommand(()->s_SwerveDrive.setHoldHeadingAngle(-xBox.getHID().getPOV() + 90))
    //   ));
      
    xBox.povUp()
      .onTrue(new InstantCommand(s_Cannon::manRotUp, s_Cannon))
      .onFalse(new InstantCommand(s_Cannon::stowMode,s_Cannon));
              
        
    xBox.povDown()
    .onTrue(new InstantCommand(s_Cannon::manRotDown, s_Cannon))
    .onFalse(new InstantCommand(s_Cannon::stowMode,s_Cannon));

    xBox.povRight()
    .onTrue(new InstantCommand(s_Cannon::manExtend, s_Cannon))
    .onFalse(new InstantCommand(s_Cannon::stowMode,s_Cannon));
    xBox.povLeft()
    .onTrue(new InstantCommand(s_Cannon::manRetract, s_Cannon))
    .onFalse(new InstantCommand(s_Cannon::stowMode,s_Cannon));
    xBox.rightTrigger(.5)
      .onTrue(new ParallelCommandGroup(new InstantCommand(s_Claw::runClawtake, s_Claw), new InstantCommand(s_Claw::closeClaw)))
      .onFalse(new InstantCommand(s_Claw::stopClawTake,s_Claw));
    // xBox.a().onTrue(new ProxyCommand(()->autoBuilder.followPathGroup(autoPathGroupOnTheFly()))
    // .beforeStarting(new InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(false))));

    // xBox.x().onTrue(new ProxyCommand(()->autoBuilder.followPathGroup(goToNearestGoal()))
    // .beforeStarting(new InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(false))));

    // xBox.y().onTrue(new InstantCommand(s_Claw::openClaw)
    //         .andThen(null)
    //         .alongWith(null)
    //         .until(null));
    }


  public Command getAutonomousCommand() {
    return 
      autoBuilder.fullAuto(autoPathGroup).withName("autoTest");
  }
  @Config(defaultValueBoolean = true)
  public void isIntegratedSteering(boolean input){
    isIntegratedSteering = input;
  }

  public ArrayList<PathPlannerTrajectory> autoPathGroupOnTheFly(){
    ArrayList<PathPlannerTrajectory> PGOTF = new ArrayList<PathPlannerTrajectory>(autoPathGroup);
    PGOTF.add(0,
    PathPlanner.generatePath(
      new PathConstraints(4, 3), 
      new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getRobotAngle()), 
      new PathPoint (autoPathGroup.get(0).getInitialHolonomicPose().getTranslation(),autoPathGroup.get(0).getInitialPose().getRotation(), autoPathGroup.get(0).getInitialHolonomicPose().getRotation())));
    
    return PGOTF; 
  }

  public ArrayList<PathPlannerTrajectory> goToNearestGoal(){
    ArrayList<PathPlannerTrajectory> PGOTF = new ArrayList<PathPlannerTrajectory>();
    if(Math.abs(leftPathGroup.get(leftPathGroup.size()-1).getEndState().poseMeters.getY() - s_SwerveDrive.getOdometryPose().getY()) < 4){
      if(Math.abs(leftPathGroup.get(leftPathGroup.size()-1).getEndState().poseMeters.getX() - s_SwerveDrive.getOdometryPose().getX()) < 8){
        PGOTF.add(PathPlanner.generatePath(
        new PathConstraints(4, 3), 
        new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getRobotAngle()), 
        new PathPoint (leftPathGroup.get(leftPathGroup.size()-1).getEndState().poseMeters.getTranslation(),leftPathGroup.get(leftPathGroup.size()-1).getEndState().poseMeters.getRotation(), leftPathGroup.get(leftPathGroup.size()-1).getEndState().holonomicRotation)));     
      } else {
        PGOTF.add(
          PathPlanner.generatePath(
            new PathConstraints(4, 3), 
            new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getRobotAngle()), 
            new PathPoint (leftPathGroup.get(0).getInitialHolonomicPose().getTranslation(),leftPathGroup.get(0).getInitialPose().getRotation(), leftPathGroup.get(0).getInitialHolonomicPose().getRotation())));
          PGOTF.addAll(leftPathGroup);
      }
    } else if(Math.abs(rightPathGroup.get(rightPathGroup.size()-1).getEndState().poseMeters.getX() - s_SwerveDrive.getOdometryPose().getX()) < 8){ 
      PGOTF.add(PathPlanner.generatePath(
        new PathConstraints(4, 3), 
        new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getRobotAngle()), 
        new PathPoint (rightPathGroup.get(rightPathGroup.size()-1).getEndState().poseMeters.getTranslation(),rightPathGroup.get(rightPathGroup.size()-1).getEndState().poseMeters.getRotation(), rightPathGroup.get(rightPathGroup.size()-1).getEndState().holonomicRotation)));     
    } else {
      PathPlanner.generatePath(
        new PathConstraints(4, 3), 
        new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getRobotAngle()), 
        new PathPoint (rightPathGroup.get(0).getInitialHolonomicPose().getTranslation(),rightPathGroup.get(0).getInitialPose().getRotation(), rightPathGroup.get(0).getInitialHolonomicPose().getRotation()));
      PGOTF.addAll(rightPathGroup);
    }

    return PGOTF;
  }
  public enum GamePieceOrientation {
    RIGHT("|>", 90),
    LEFT("<|", -90), 
    UPRIGHT("/\\", 0), 
    CUBE("口", 0); 

    private String friendlyName; 
    private double rotOrientationAngle;

    public String getFriendlyName() {
      return friendlyName;
    }
    public double getRotOrientForRoto(){
      return rotOrientationAngle; 
    }

    private GamePieceOrientation(String friendlyName, double rotOrientationAngle) {
      this.friendlyName = friendlyName;
      this.rotOrientationAngle = rotOrientationAngle;
    }
  
  }

  public enum NodeDriverStation {
    ONE("driver station one"), TWO("driver station two"), THREE("driver station three");
    public final String dsFriendlyName;
    private NodeDriverStation(String dsFriendlyName) {
      this.dsFriendlyName = dsFriendlyName;
    }
  }
}

