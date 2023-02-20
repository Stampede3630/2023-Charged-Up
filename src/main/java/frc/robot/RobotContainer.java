// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NodePosition.NodeGrid;
import frc.robot.NodePosition.NodeGroup;
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
      .withWidget("GridChooserWidget");

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
    // // new InstantCommand(s_Cannon::stowMode)
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

    xBox.b()
        .onTrue(new InstantCommand(() -> s_SwerveDrive.setHoldHeadingFlag(false)));

    // xBox.povCenter().negate().onTrue(
    // new SequentialCommandGroup(
    // new InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(true)),
    // new
    // InstantCommand(()->s_SwerveDrive.setHoldHeadingAngle(-xBox.getHID().getPOV()
    // + 90))
    // ));

    xBox.povUp()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRotUp)));
    xBox.povDown()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRotDown)));
    xBox.povRight()
        .whileTrue(new RepeatCommand(new InstantCommand(s_Cannon::manExtend, s_Cannon)));
    xBox.povLeft()
        .whileTrue(new RepeatCommand(new InstantCommand(s_Cannon::manRetract, s_Cannon)));
    xBox.rightTrigger(.5)
        .onTrue(new InstantCommand(s_Claw::runClawtake, s_Claw))
          // .alongWith(Commands.runOnce(() -> s_Cannon.setCannonAngleSides(cannonFacing(), -20))))
        .onFalse((new InstantCommand(s_Claw::stopClawTake, s_Claw)));
    xBox.leftTrigger(.5)
        .onTrue(new InstantCommand(s_Claw::reverseClawtake, s_Claw))
        .onFalse(new InstantCommand(s_Claw::stopClawTake, s_Claw));
    xBox.rightBumper()
        .onTrue(new InstantCommand(s_Claw::openClaw, s_Claw))
        .onFalse(new InstantCommand(s_Claw::stopClawTake, s_Claw));
    xBox.leftBumper()
        .onTrue(new InstantCommand(s_Claw::closeClaw, s_Claw))
        .onFalse(new InstantCommand(s_Claw::stopClawTake, s_Claw));
    // xBox.a().debounce(.1)
    //     .onTrue(new InstantCommand(s_Claw::rotoClaw))
    //     .onFalse(new InstantCommand(s_Claw::stopClawTake));
    // xBox.x().debounce(.1)
    //     .onTrue(new InstantCommand(s_Claw::rotoClawReverse))
    //     .onFalse(new InstantCommand(s_Claw::stopClawTake));
    xBox.x().debounce(.1)
      .onTrue(Commands.runOnce(s_Claw::flipRotoClawtake).andThen(new PrintCommand("flipclawtake")));
    // xBox.y()
    //     .onTrue((Commands.runOnce(()-> s_Cannon.setCannonAngle(armTestSetPoints.getSelected().getTestCannonAngle())))
    //     .andThen(new SequentialCommandGroup(Commands.waitUntil(s_Cannon::errorWithinRange)),
    //       Commands.runOnce(()-> s_Cannon.setExtensionInches(armTestSetPoints.getSelected().getTestCannonExtension()))));

    // new Trigger(s_Claw::haveGamePiece)
    //   .whileTrue(new RepeatCommand(Commands.runOnce(() -> s_Cannon.setCannonAngleSides(cannonFacing(), 30)))
    //     .andThen(Commands.runOnce(() -> s_Claw.flipRotoClawtake()))
    //     .ignoringDisable(true))
    //   .whileFalse(new RepeatCommand(Commands.runOnce(() -> s_Cannon.setCannonAngleSides(cannonFacing(), 150))
    //     .andThen(Commands.runOnce(() -> s_Claw.prepareForGamePiece(gamePieceOrientationChooser.getSelected()))))
    //     .ignoringDisable(true));

    new Trigger(() -> robotFacing() != FacingPOI.NOTHING)
      .onTrue(Commands.either(
        Commands.runOnce(() -> s_Claw.faceCommunitySides(s_Cannon.setCannonAngleSides(robotFacing(), 150)))
        .unless(xBox.rightTrigger(.75))  //towards community
        .ignoringDisable(true),
        Commands.runOnce(() -> s_Cannon.setCannonAngleSides(robotFacing(), 30)).unless(xBox.rightTrigger(.75)) // towards pickup
        .andThen(Commands.runOnce(() -> s_Claw.prepareForGamePiece(gamePieceOrientationChooser.getSelected())))
        .ignoringDisable(true), 
        s_Claw::haveGamePiece));


    // xBox.a().onTrue(new
    // ProxyCommand(()->autoBuilder.followPathGroup(autoPathGroupOnTheFly()))
    // .beforeStarting(new
    // InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(false))));
    //
    // xBox.x().onTrue(new
    // ProxyCommand(()->autoBuilder.followPathGroup(goToNearestGoal()))
    // .beforeStarting(new
    // InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(false))));
    //
    // xBox.y().onTrue(new InstantCommand(s_Claw::openClaw)
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
            new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getRobotAngle()),
            new PathPoint(autoPathGroup.get(0).getInitialHolonomicPose().getTranslation(),
                autoPathGroup.get(0).getInitialPose().getRotation(),
                autoPathGroup.get(0).getInitialHolonomicPose().getRotation())));

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
              rightPathGroup.get(0).getInitialPose().getRotation(),
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
    CONE, CUBE
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

}
