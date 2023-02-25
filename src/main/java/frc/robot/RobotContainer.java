// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CompletionException;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NodePosition.NodeGrid;
import frc.robot.NodePosition.NodeGroup;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.RotoClawtake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TheCannon;
import frc.robot.subsystems.swerve.SwerveConstants;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controller setup. For simulations google: x360CE */
  private final CommandXboxController xBox = new CommandXboxController(0);

  private boolean isIntegratedSteering = true;
  SwerveAutoBuilder autoBuilder;
  ArrayList<PathPlannerTrajectory> autoPathGroup, leftPathGroup, rightPathGroup;
  
  // This is just an example event map. It would be better to have a constant,
  // global event map
  // in your code that will be used by all path following commands.

  // This is just an example event map. It would be better to have a constant,
  // global event map

  SendableChooser<NodeDriverStation> nodeDriverStation = new SendableChooser<>();
  SendableChooser<ArmTestSetPoints> armTestSetPoints = new SendableChooser<>();
  SendableChooser<GamePieceOrientation> gamePieceOrientationChooser = new SendableChooser<>();
  SendableChooser<frc.robot.NodePosition.NodeGroup> nodeGroupChooser = new SendableChooser<>();
  SendableChooser<NodeGrid> nodeGridChooser = new SendableChooser<>();

  @Log
  private final SwerveDrive s_SwerveDrive = new SwerveDrive();
  // The robot's subsystems and commands are defined here...
  private final RotoClawtake s_Claw = new RotoClawtake();

  private final TheCannon s_Cannon = new TheCannon();

  private final LEDs s_LEDs = new LEDs();
  
  public GamePieceOrientation previousGamePieceOrientation;
  public GamePieceType gamePieceType;
  // private GamePieceType prev;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  NetworkTable datatable = inst.getTable("GamePieceOrientationChooser");

  public String gamePieceString = gamePieceOrientationChooser.getSelected().getGamePieceType().toString();

  
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    for (ArmTestSetPoints setPoint : ArmTestSetPoints.values()) {
      armTestSetPoints.addOption(setPoint.testPointName, setPoint);
    }
    for (NodeDriverStation ds : NodeDriverStation.values()) {
      nodeDriverStation.addOption(ds.dsFriendlyName, ds);
    }
    /**
     * Preferences are cool. they store the values in the roborio flash memory so
     * they don't necessarily get reset to default.
     */
    Preferences.initBoolean("pFieldRelative", Constants.fieldRelative);
    Preferences.initBoolean("pAccelInputs", Constants.acceleratedInputs);
    Preferences.initDouble("pDriveGovernor", Constants.driveGovernor);
    Preferences.initBoolean("pOptimizeSteering", SwerveConstants.OPTIMIZESTEERING);
    Preferences.initDouble("pKPRotationController", SwerveConstants.kPRotationController);
    Preferences.initDouble("pKIRotationController", SwerveConstants.kDRotationController);
    Preferences.initDouble("pKDRotationController", SwerveConstants.kIRotationController);
    Preferences.initBoolean("pIntSteering", true);
    Preferences.initDouble("CannonKP", 1.0 / 30.0);
    Preferences.initDouble("CannonKI", 0.0);
    Preferences.initDouble("CannonKD", 0.0);
    Preferences.initDouble("ExtensionKP", 1.0 / 6.0);
    Preferences.initDouble("ExtensionKI", 0.0);
    Preferences.initDouble("ExtensionPD", 0.0);
    Preferences.initDouble("RotoClawKP", 6.0 / Math.PI);
    Preferences.initDouble("RotoClawKI", 0.0);
    Preferences.initDouble("RotoClawPD", 0.0);
    Preferences.initDouble("ClawKP", 6.0 / Math.PI);
    Preferences.initDouble("ClawKI", 0.0);
    Preferences.initDouble("ClawKD", 0.0);
    Preferences.initBoolean("Wanna PID Roto", false);
    Preferences.initBoolean("Wanna PID Cannon", false);


    autoPathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("swerveTest",
        new PathConstraints(2, 2));

    for (GamePieceOrientation orientation : GamePieceOrientation.values()) {
      gamePieceOrientationChooser.addOption(orientation.getFriendlyName(), orientation);
    }
    gamePieceOrientationChooser.setDefaultOption(GamePieceOrientation.UPRIGHT.getFriendlyName(), GamePieceOrientation.UPRIGHT);
    for (NodeGroup group : NodeGroup.values()) {
      nodeGroupChooser.addOption(group.name(), group);
    }
    nodeGroupChooser.setDefaultOption(NodeGroup.CENTER.name(), NodeGroup.CENTER);
    for (NodeGrid child : NodeGrid.values()) {
      nodeGridChooser.addOption(child.name(), child);
    }
    nodeGridChooser.setDefaultOption(NodeGrid.MID_CENTER.name(), NodeGrid.MID_CENTER);

    Shuffleboard.getTab("nodeSelector")
        .add("orientation", gamePieceOrientationChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);

    Shuffleboard.getTab("nodeSelector")
        .add("Node Driver Station", nodeDriverStation)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
      
    Shuffleboard.getTab("nodeSelector")
        .add("Arm Point Select", armTestSetPoints)
        .withWidget(BuiltInWidgets.kComboBoxChooser);

    Shuffleboard.getTab("nodeSelector")
      .add("Node Group Chooser", nodeGroupChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser);
      
    Shuffleboard.getTab("nodeSelector")
      .add("Node Grid Chooser", nodeGridChooser)
      .withWidget("GridChooserWidget")
      .withProperties(Map.of("Cone Image","C:\\Users\\Mustangs\\Documents\\evanArtwork\\cone.png",
      "Cube Image","C:\\Users\\Mustangs\\Documents\\evanArtwork\\cube.png",
      "Hybrid Image","C:\\Users\\Mustangs\\Documents\\evanArtwork\\hybrid.png",
      "R", 1));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("1stBallPickup", new WaitCommand(2));
    eventMap.put("2ndBallPickup", new WaitCommand(2));
    eventMap.put("3rdBallPickup", new WaitCommand(2));

    autoBuilder = new SwerveAutoBuilder(
        s_SwerveDrive::getOdometryPose, // Pose2d supplier
        s_SwerveDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        s_SwerveDrive.getKinematics(), // SwerveDriveKinematics
        new PIDConstants(5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
                                       // PID controllers)
        new PIDConstants(5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
                                       // controller)
        s_SwerveDrive::setAutoModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        s_SwerveDrive // The drive subsystem. Used to properly set the requirements of path following
                      // commands
    );

    s_SwerveDrive.setDefaultCommand(
        s_SwerveDrive.joystickDriveCommand(
            xBox::getLeftY,
            xBox::getLeftX,
            xBox::getRightX).withName("DefaultDrive"));

    // // s_Cannon.setDefaultCommand(
    // // Commands.runOnce(s_Cannon::stowMode)
    // );

    // This will load the file "FullAuto.path" and generate it with a max velocity
    // of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    // autoPathGroup = ;//PathPlanner.loadPathGroup("Test5Ball", new
    // PathConstraints(4, 3));
    // leftPathGroup = ;//PathPlanner.loadPathGroup("LeftPath1WP", new
    // PathConstraints(4, 3));
    // rightPathGroup = ;//PathPlanner.loadPathGroup("RightPath1WP", new
    // PathConstraints(4, 3));
    // Configure the button bindings
    configureButtonBindings();
    Logger.configureLoggingAndConfig(this, false);
    // SmartDashboard.getString("nodeselector/Node Grid Chooser", "");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(() -> Preferences.getBoolean("pIntSteering", true))
        .onFalse(s_SwerveDrive.switchToRemoteSteerCommand().ignoringDisable(true))
        .onTrue(s_SwerveDrive.switchToIntegratedSteerCommand().ignoringDisable(true));

    /**
     * Trigger Coast/Brake modes when DS is Disabled/Enabled.
     * Trigger runs WHILETRUE for coast mode. Coast Mode method
     * is written to wait for slow speeds before setting to coast
     */
    // new Trigger(()-> s_Claw.heldGamePiece == GamePieceType.CONE)
    //   .onTrue(s_LEDs::becomeYellow)
    //   .onFalse(s_LEDs::becomeRainbow);

    new Trigger(DriverStation::isDisabled)
        .whileTrue(s_SwerveDrive.setToCoast()
            .ignoringDisable(true)
            .withName("SetToCoast"))
        .onFalse(s_SwerveDrive.setToBrake()
            .withName("setToBrake"));

    /**
     * next two triggers are to "toggle" rotation HOLD mode and set a heading
     */

    new Trigger(() -> Math.abs(xBox.getRightX()) < .1)
        .and(s_SwerveDrive::getHoldHeadingFlag)
        .and(new Trigger(s_SwerveDrive::getAtGoal).negate())
        .whileTrue(
            s_SwerveDrive.holdHeadingCommand(
                xBox::getLeftY,
                xBox::getLeftX)
                .withName("holdHeading"))
        .whileFalse(
            s_SwerveDrive.joystickDriveCommand(
                xBox::getLeftY,
                xBox::getLeftX,
                xBox::getRightX)
                .withName("StandardOperatorDrive"));

    // xBox.b()
    //     .onTrue(Commands.runOnce(() -> s_SwerveDrive.setHoldHeadingFlag(false)));
    

    // xBox.povCenter().negate().onTrue(
    // new SequentialCommandGroup(
    // Commands.runOnce(()->s_SwerveDrive.setHoldHeadingFlag(true)),
    // new
    // InstantCommand(()->s_SwerveDrive.setHoldHeadingAngle(-xBox.getHID().getPOV()
    // + 90))
    // ));

    xBox.leftBumper().onTrue(new PrintCommand("bumper").ignoringDisable(true));

    xBox.povUp()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRotUp)));
    xBox.povDown()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRotDown)));
    xBox.povRight()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manExtend, s_Cannon)));
    xBox.povLeft()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRetract, s_Cannon)));
    xBox.rightTrigger(.55).debounce(.1, DebounceType.kFalling)
        .onTrue(Commands.runOnce(s_Claw::runClawtake)
          .alongWith(Commands.runOnce(() -> s_Cannon.setCannonAngleSides(robotFacing(), -14.0)))
          .alongWith(Commands.runOnce(s_Claw::closeClaw)))
        .onFalse(Commands.runOnce(s_Claw::stopClawTake)
          .andThen(Commands.either(
            Commands.runOnce(() -> s_Claw.faceCommunitySides(s_Cannon.setCannonAngleSides(robotFacing(), 140)))
            .unless(xBox.rightTrigger(.5)),
            Commands.runOnce(() -> s_Cannon.setCannonAngleSides(robotFacing(), 40)).unless(xBox.rightTrigger(.5)) // towards pickup
            .andThen(Commands.runOnce(() -> s_Claw.prepareForGamePiece(gamePieceOrientationChooser.getSelected()))), 
            s_Claw::haveGamePiece)));
    xBox.leftTrigger(.55).debounce(.1, DebounceType.kFalling)
        .onTrue(Commands.runOnce(s_Claw::openClaw)
          .alongWith(Commands.runOnce(s_Claw::reverseClawtake)))
        .onFalse(Commands.runOnce(s_Claw::stopClawTake)
          .andThen(Commands.runOnce(()-> s_Cannon.setExtensionInches(5.0))
          .andThen(Commands.waitUntil(s_Cannon::extensionErrorWithinRange)
          .andThen(Commands.runOnce(()-> s_Cannon.setCannonAngleSides(robotFacing(), 150))))));
    xBox.rightBumper().debounce(.1, DebounceType.kFalling)
        .onTrue(Commands.runOnce(s_Claw::openClaw).andThen(new PrintCommand("Right Bumper")))
        .onFalse(Commands.runOnce(s_Claw::stopClawMotor));
    xBox.leftBumper().debounce(.1, DebounceType.kFalling)
        .onTrue(Commands.runOnce(s_Claw::closeClaw).andThen(new PrintCommand("Left Bumper")))
        .onFalse(Commands.runOnce(s_Claw::stopClawMotor));

    new Trigger(s_Claw::haveGamePiece)
      .onTrue(Commands.runOnce(s_Claw::closeClaw)
        .andThen(s_Claw::slowClawTake)
        .andThen(()-> s_Cannon.setCannonAngleSides(robotFacing(), 140))
        .andThen(s_LEDs::beWhoYouAre));

      
    // xBox.a().debounce(.1)
    //     .onTrue(Commands.runOnce(s_Claw::rotoClaw))
    //     .onFalse(Commands.runOnce(s_Claw::stopClawTake));
    // xBox.x().debounce(.1)
    //     .onTrue(Commands.runOnce(s_Claw::rotoClawReverse))
    //     .onFalse(Commands.runOnce(s_Claw::stopClawTake));
    // xBox.x().debounce(.1)
    //   .onTrue(Commands.runOnce(s_Claw::flipRotoClawtake).andThen(new PrintCommand("flipclawtake")));
    xBox.y()
        .onTrue((Commands.runOnce(()-> s_Cannon.setCannonAngleSides(robotFacing(), NodePosition.getNodePosition(nodeGroupChooser.getSelected(), nodeGridChooser.getSelected()).getCannonAngle())))
        .andThen(new SequentialCommandGroup(Commands.waitUntil(s_Cannon::cannonErrorWithinRange)),
          Commands.runOnce(()-> s_Cannon.setExtensionInches(NodePosition.getNodePosition(nodeGroupChooser.getSelected(), nodeGridChooser.getSelected()).getExtension()))));
    
    // new Trigger(s_Claw::haveGamePiece)
    //   .whileTrue(new RepeatCommand(Commands.runOnce(() -> s_Cannon.setCannonAngleSides(cannonFacing(), 30)))
    //     .andThen(Commands.runOnce(() -> s_Claw.flipRotoClawtake()))
    //     .ignoringDisable(true))
    //   .whileFalse(new RepeatCommand(Commands.runOnce(() -> s_Cannon.setCannonAngleSides(cannonFacing(), 150))
    //     .andThen(Commands.runOnce(() -> s_Claw.prepareForGamePiece(gamePieceOrientationChooser.getSelected()))))
    //     .ignoringDisable(true));

    new Trigger(() -> robotFacing() != FacingPOI.NOTHING)
      .onTrue(Commands.either(
        Commands.runOnce(() -> s_Claw.faceCommunitySides(s_Cannon.setCannonAngleSides(robotFacing(), 140)))
        .unless(xBox.rightTrigger(.5)),
        Commands.runOnce(() -> s_Cannon.setCannonAngleSides(robotFacing(), 40)).unless(xBox.rightTrigger(.5)) // towards pickup
        .andThen(Commands.runOnce(() -> s_Claw.prepareForGamePiece(gamePieceOrientationChooser.getSelected()))), 
        s_Claw::haveGamePiece));
  
    xBox.a().onTrue(new
    ProxyCommand(()->autoBuilder.followPathGroup(autoPathGroupOnTheFly()))
    .beforeStarting(new
    InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(false))));

    new Trigger(()->listenForOrientationChange())
      .onTrue(Commands.runOnce(()-> s_Claw.setRotoAngle(previousGamePieceOrientation.getRotOrientForRoto()))
        .andThen(Commands.runOnce(s_LEDs::bePurple)
        .unless(()->(isCone() || isNoThing())))
        .andThen(Commands.runOnce(s_LEDs::beYellow)
        .unless(()->(isCube() || isNoThing())))
        .andThen(Commands.runOnce(s_LEDs::beWhoYouAre)
        .unless(()->(isCone() || isCube()))));

      // .unless(()-> s_Claw.haveGamePiece())
      // .andThen(()->s_Claw.setRotoAngle(previousGamePieceOrientation.getRotOrientForRoto())));

    //
    // xBox.x().onTrue(new
    // ProxyCommand(()->autoBuilder.followPathGroup(goToNearestGoal()))
    // .beforeStarting(new
    // InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(false))));
    //
    // xBox.y().onTrue(Commands.runOnce(s_Claw::openClaw)
    // .andThen(null)
    // .alongWith(null)
    // .until(null));

  }

  public Command getAutonomousCommand() {
    return autoBuilder.fullAuto(autoPathGroup).withName("autoTest");
  }

  @Config(defaultValueBoolean = true)
  public void isIntegratedSteering(boolean input) {
    isIntegratedSteering = input;
  }

  public ArrayList<PathPlannerTrajectory> autoPathGroupOnTheFly() {
    ArrayList<PathPlannerTrajectory> PGOTF = new ArrayList<PathPlannerTrajectory>(autoPathGroup);
    PGOTF.add(0,
        PathPlanner.generatePath(
            new PathConstraints(4, 3),
            new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getOdometryPose().getRotation()),
            new PathPoint(new Translation2d(Units.inchesToMeters(nodeGroupChooser.getSelected().xCoord), 
              Units.inchesToMeters(nodeGroupChooser.getSelected().yCoord) + Units.inchesToMeters(getYOffset())), 
              new Rotation2d(robotFacing() == FacingPOI.COMMUNITY ? 0:180), 
              new Rotation2d(robotFacing() == FacingPOI.COMMUNITY ? 0:180))));

    return PGOTF;
  }

  public ArrayList<PathPlannerTrajectory> goToNearestGoal() {
    ArrayList<PathPlannerTrajectory> PGOTF = new ArrayList<PathPlannerTrajectory>();
    if (Math.abs(leftPathGroup.get(leftPathGroup.size() - 1).getEndState().poseMeters.getY()
        - s_SwerveDrive.getOdometryPose().getY()) < 4) {
      if (Math.abs(leftPathGroup.get(leftPathGroup.size() - 1).getEndState().poseMeters.getX()
          - s_SwerveDrive.getOdometryPose().getX()) < 8) {
        PGOTF.add(PathPlanner.generatePath(
            new PathConstraints(4, 3),
            new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getRobotAngle()),
            new PathPoint(leftPathGroup.get(leftPathGroup.size() - 1).getEndState().poseMeters.getTranslation(),
                leftPathGroup.get(leftPathGroup.size() - 1).getEndState().poseMeters.getRotation(),
                leftPathGroup.get(leftPathGroup.size() - 1).getEndState().holonomicRotation)));
      } else {
        PGOTF.add(
            PathPlanner.generatePath(
                new PathConstraints(4, 3),
                new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getRobotAngle()),
                new PathPoint(leftPathGroup.get(0).getInitialHolonomicPose().getTranslation(),
                    leftPathGroup.get(0).getInitialPose().getRotation(),
                    leftPathGroup.get(0).getInitialHolonomicPose().getRotation())));
        PGOTF.addAll(leftPathGroup);
      }
    } else if (Math.abs(rightPathGroup.get(rightPathGroup.size() - 1).getEndState().poseMeters.getX()
        - s_SwerveDrive.getOdometryPose().getX()) < 8) {
      PGOTF.add(PathPlanner.generatePath(
          new PathConstraints(4, 3),
          new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getRobotAngle()),
          new PathPoint(rightPathGroup.get(rightPathGroup.size() - 1).getEndState().poseMeters.getTranslation(),
              rightPathGroup.get(rightPathGroup.size() - 1).getEndState().poseMeters.getRotation(),
              rightPathGroup.get(rightPathGroup.size() - 1).getEndState().holonomicRotation)));
    } else {
      PathPlanner.generatePath(
          new PathConstraints(4, 3),
          new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getRobotAngle()),
          new PathPoint(rightPathGroup.get(0).getInitialHolonomicPose().getTranslation(),
              calculateHeading(),
              rightPathGroup.get(0).getInitialHolonomicPose().getRotation()));
      PGOTF.addAll(rightPathGroup);
    }

    return PGOTF;
  }

  @Log
  public double getYOffset() {
    return NodePosition.getNodePosition(nodeGroupChooser.getSelected(), nodeGridChooser.getSelected()).getYCoord();
  }

  public FacingPOI robotFacing() {
    FacingPOI gyroFacing = FacingPOI.NOTHING;
    if (Math.abs(MathUtil.inputModulus(s_SwerveDrive.getRobotAngle().getDegrees(), -180, 180))<50) // gyro facing community
      gyroFacing = FacingPOI.COMMUNITY;
    else if (Math.abs(MathUtil.inputModulus(s_SwerveDrive.getRobotAngle().getDegrees(), -180, 180))>130) // gyro facing HP
      gyroFacing = FacingPOI.HUMAN_PLAYER;
    return gyroFacing;
  }

  @Log
  public String robotFacingString() {
    return robotFacing().toString();
  }

  @Log
  public String cannonFacingString() {
    return cannonFacing().toString();
  }

  public FacingPOI cannonFacing() {
    FacingPOI gyroFacing = robotFacing();
    boolean cannonFacingGyroZero = s_Cannon.getCannonAngleEncoder() < 90;

    if (gyroFacing == FacingPOI.NOTHING)
      return FacingPOI.NOTHING;

    if (gyroFacing == FacingPOI.COMMUNITY)
      if (cannonFacingGyroZero)
        return FacingPOI.COMMUNITY;
      else
        return FacingPOI.HUMAN_PLAYER;
    else // gyro must be facing HP
      if (cannonFacingGyroZero)
        return FacingPOI.HUMAN_PLAYER;
      else
        return FacingPOI.COMMUNITY;
  }

  public enum FacingPOI {
    COMMUNITY, HUMAN_PLAYER, NOTHING
  }

  public enum GamePieceOrientation {
    RIGHT("|>", 90, GamePieceType.CONE),
    LEFT("<|", -90, GamePieceType.CONE),
    UPRIGHT("/\\", 0, GamePieceType.CONE),
    CUBE("口", 90, GamePieceType.CUBE);

    private String friendlyName;
    private double rotOrientationAngle;
    private GamePieceType gamePieceType;

    public String getFriendlyName() {
      return friendlyName;
    }

    public double getRotOrientForRoto() {
      return rotOrientationAngle;
    }

    public GamePieceType getGamePieceType() {
      return gamePieceType;
    }

    private GamePieceOrientation(String friendlyName, double rotOrientationAngle, GamePieceType gamePieceType) {
      this.friendlyName = friendlyName;
      this.rotOrientationAngle = rotOrientationAngle;
      this.gamePieceType = gamePieceType;
    }

  }

  public enum GamePieceType {
    CONE, CUBE, NOTHING
  }

  public enum NodeDriverStation {
    ONE("driver station one"), TWO("driver station two"), THREE("driver station three");

    public final String dsFriendlyName;

    private NodeDriverStation(String dsFriendlyName) {
      this.dsFriendlyName = dsFriendlyName;
    }
  }

  public static enum ArmTestSetPoints {
    HIGH("for high node", 40, 40),
    MID("for mid node", 40, 20),
    LOW("for low/hybrid node", 0, 10),
    UP("straight up", 90, 0),
    ZERO("straight out", 0, 0);

    public final String testPointName;
    public final double testCannonAngle;
    public final double testCannonExtention;

    private ArmTestSetPoints(String testPointName, double testCannonAngle, double testCannonExtention){
      this.testPointName = testPointName;
      this.testCannonAngle = testCannonAngle;
      this.testCannonExtention = testCannonExtention;
    }

    public double getTestCannonExtension(){
      return testCannonExtention;
    }

    public double getTestCannonAngle(){
      return testCannonAngle;
    }

  }

  public double armTestSetPointsAngle(SendableChooser<ArmTestSetPoints> armTestSetPoints){
  
    return armTestSetPoints.getSelected().getTestCannonAngle();
  }

  public boolean listenForOrientationChange(){

    previousGamePieceOrientation = gamePieceOrientationChooser.getSelected();

    if (gamePieceOrientationChooser.getSelected() != previousGamePieceOrientation) {
      previousGamePieceOrientation = gamePieceOrientationChooser.getSelected();
      System.out.println("a thing happened");
      return true;
    } else{
      System.out.println("a thing did not happen");
      return false;
    }
  }

  public Rotation2d calculateHeading(){
    Pose2d roboTranslationX = s_SwerveDrive.getOdometryPose();
    double desiredTranslationX = (Units.inchesToMeters(nodeGroupChooser.getSelected().xCoord));
    double desiredTranslationY = Units.inchesToMeters(nodeGroupChooser.getSelected().yCoord + getYOffset());

    double oOfOA = roboTranslationX.getX() - desiredTranslationX;
    double aOfOA = roboTranslationX.getY() - desiredTranslationY;

    return new Rotation2d(-Math.atan(oOfOA/aOfOA));
  }

  public boolean listenForGamePieceLED(){

    gamePieceType = gamePieceOrientationChooser.getSelected().getGamePieceType();

    if (gamePieceOrientationChooser.getSelected().getGamePieceType() != gamePieceType) {
      gamePieceType = gamePieceOrientationChooser.getSelected().getGamePieceType();
      System.out.println("a thing happened");
      return true;
    } else{
      System.out.println("a thing did not happen");
      return false;
    }
  }

  public boolean isCone(){
    if (gamePieceString.equals("CONE")){
      return true;
    } else {
      return false;
    }
  }

  public boolean isCube(){
    if (gamePieceString.equals("CUBE")){
      return true;
    } else {
      return false;
    }
  }

  public boolean isNoThing(){
    if (gamePieceString.equals("NOTHING")){
      return true;
    } else {
      return false;
    }
    //Not necessary because isNoThing is never true 6:13 PM Friday, February 24 (=^-w-^=), :3, UwU, >wO
  }

  // public boolean beDesiredPieceColorR(){
  //   if (gamePieceType == gamePieceOrientationChooser.getSelected().getGamePieceType().CONE){
  //     return false;
  //   } else {
  //     return true;
  //   }
  // }
}
