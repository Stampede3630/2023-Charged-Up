// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NodePosition.NodeGrid;
import frc.robot.NodePosition.NodeGroup;
import static frc.robot.Constants.*;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Lid;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TheCannon;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.util.SendableChooserWrapper;
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

  SwerveAutoBuilder autoBuilder;
  public final String[] AUTOS_TO_LOAD = {"chargeSimple", "onePieceConeGrab","onePieceCubeGrab","twoPieceCube","twoPieceCone","LZTwoPieceCone","LZTwoPieceCube","LZTwoPieceCubeCharge"};


  // This is just an example event map. It would be better to have a constant,
  // global event map
  // in your code that will be used by all path following commands.

  // This is just an example event map. It would be better to have a constant,
  // global event map

  SendableChooser<NodeDriverStation> nodeDriverStation = new SendableChooser<>();
  SendableChooser<ScoringSetPoints> scoringSetPoints = new SendableChooser<>();
  SendableChooserWrapper<GamePieceType> gamePieceTypeChooser = new SendableChooserWrapper<>();
  SendableChooser<frc.robot.NodePosition.NodeGroup> nodeGroupChooser = new SendableChooser<>();
  SendableChooser<NodeGrid> nodeGridChooser = new SendableChooser<>();
  SendableChooser<PickupLocation> pickupLocationChooser = new SendableChooser<>();
  SendableChooser<List<PathPlannerTrajectory>> autoSelect = new SendableChooser<>();
  PowerDistribution pdh = new PowerDistribution(1,ModuleType.kRev);

  @Log
  private final SwerveDrive s_SwerveDrive = new SwerveDrive();
  // The robot's subsystems and commands are defined here...
  private final TheCannon s_Cannon = new TheCannon();

  private final LEDs s_LEDs = new LEDs();
  private final Lid s_Lid = new Lid();
  private final Intake s_Intake = new Intake();
  
  public double intakeCannonAngle;
  public double intakeLidAngle;
  @Log
  public double intakeSpeed;
  public boolean facingOverrideButton;
  // private GamePieceType prev;  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    for (ScoringSetPoints setPoint : ScoringSetPoints.values()) {
      scoringSetPoints.addOption(setPoint.setPointName, setPoint);
    }
    for (NodeDriverStation ds : NodeDriverStation.values()) {
      nodeDriverStation.addOption(ds.dsFriendlyName, ds);
    }

    
    /**
     * Preferences are cool. they store the values in the roborio flash memory so
     * they don't necessarily get reset to default.
     */
    Preferences.initBoolean("pFieldRelative", DriverConstants.FIELD_RELATIVE);
    Preferences.initBoolean("pAccelInputs", DriverConstants.ACCELERATED_INPUTS);
    Preferences.initDouble("pDriveGovernor", DriverConstants.DRIVE_GOVERNOR);
    Preferences.initBoolean("pOptimizeSteering", SwerveConstants.OPTIMIZESTEERING);
    Preferences.initDouble("pKPRotationController", SwerveConstants.kPRotationController);
    Preferences.initDouble("pKIRotationController", SwerveConstants.kDRotationController);
    Preferences.initDouble("pKDRotationController", SwerveConstants.kIRotationController);
    Preferences.initDouble("CannonKP", CannonConstants.KP);
    Preferences.initDouble("CannonKI", CannonConstants.KI);
    Preferences.initDouble("CannonKD", CannonConstants.KD);
    Preferences.initDouble("ExtensionKP", ExtendoConstants.KP);
    Preferences.initDouble("ExtensionKI", ExtendoConstants.KI);
    Preferences.initDouble("ExtensionPD", ExtendoConstants.KD);

    Preferences.initDouble("LidKP", LidConstants.KP);
    Preferences.initDouble("LidKI", LidConstants.KI);
    Preferences.initDouble("LidKD", LidConstants.KD);
    Preferences.initBoolean("Wanna PID Roto", false);
    Preferences.initBoolean("Wanna PID Cannon", false);
    Preferences.initBoolean("Wanna PID Lid", false);

    Shuffleboard.getTab("pdh")
      .add("PDH", pdh)
      .withWidget(BuiltInWidgets.kPowerDistribution);



    
    for (GamePieceType orientation : GamePieceType.values()) {
      gamePieceTypeChooser.addOption(orientation.name(), orientation);
    }
    gamePieceTypeChooser.setDefaultOption(GamePieceType.TIPPED_CONE.name(), GamePieceType.TIPPED_CONE);
    for (NodeGroup group : NodeGroup.values()) {
      nodeGroupChooser.addOption(group.name(), group);
    }
    nodeGroupChooser.setDefaultOption(NodeGroup.CENTER.name(), NodeGroup.CENTER);
    for (NodeGrid child : NodeGrid.values()) {
      nodeGridChooser.addOption(child.name(), child);
    }
    nodeGridChooser.setDefaultOption(NodeGrid.MID_CENTER.name(), NodeGrid.MID_CENTER);
    for (PickupLocation loc : PickupLocation.values()) {
      pickupLocationChooser.addOption(loc.name(), loc);
    }
    pickupLocationChooser.setDefaultOption(PickupLocation.GROUND.name(), PickupLocation.GROUND);

    Shuffleboard.getTab("nodeSelector")
        .add("orientation", gamePieceTypeChooser)
        .withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withPosition(0, 0)
        .withSize(8, 3);

    Shuffleboard.getTab("nodeSelector")
        .add("Auto Selector", autoSelect)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(17, 0)
        .withSize(3, 1);

    Shuffleboard.getTab("nodeSelector")
      .add("Node Group Chooser", nodeGroupChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withPosition(0, 6)
      .withSize(8, 3);
      
    Shuffleboard.getTab("nodeSelector")
      .add("Node Grid Chooser", nodeGridChooser)
      .withWidget("GridChooserWidget")
      .withProperties(Map.of("Cone Image","C:\\Users\\Mustangs\\Documents\\evanArtwork\\cone.png",
      "Cube Image","C:\\Users\\Mustangs\\Documents\\evanArtwork\\cube.png",
      "Hybrid Image","C:\\Users\\Mustangs\\Documents\\evanArtwork\\hybrid.png",
      "R", 1))
      .withPosition(8, 0)
      .withSize(9, 9);

    Shuffleboard.getTab("nodeSelector")
      .add("Pickup Location", pickupLocationChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withPosition(0, 3)
      .withSize(8, 3);

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("1stBallPickup", new WaitCommand(2));
    eventMap.put("2ndBallPickup", new WaitCommand(2));
    eventMap.put("3rdBallPickup", new WaitCommand(2));

    autoBuilder = new SwerveAutoBuilder(
        s_SwerveDrive::getOdometryPose, // Pose2d supplier
        s_SwerveDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        s_SwerveDrive.getKinematics(), // SwerveDriveKinematics
        new PIDConstants(SwerveConstants.kP, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
                                       // PID controllers)
        new PIDConstants(1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
                                       // controller)
        s_SwerveDrive::setAutoModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        s_SwerveDrive // The drive subsystem. Used to properly set the requirements of path following
                      // commands
    );

    for (String autoName : AUTOS_TO_LOAD) { // load all autos dynamically
      autoSelect.addOption(autoName, PathPlanner.loadPathGroup(autoName,
        new PathConstraints(.5, .5)));
    }


    s_SwerveDrive.setDefaultCommand(
        s_SwerveDrive.joystickDriveCommand(
            xBox::getLeftY,
            xBox::getLeftX,
            xBox::getRightX).withName("DefaultDrive"));

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

    /**
     * Trigger Coast/Brake modes when DS is Disabled/Enabled.
     * Trigger runs WHILETRUE for coast mode. Coast Mode method
     * is written to wait for slow speeds before setting to coast
     */

    new Trigger(DriverStation::isDisabled)
        .whileTrue(Commands.repeatingSequence(Commands.runOnce(s_SwerveDrive::setToCoast)).ignoringDisable(true))
        .whileTrue((Commands.runOnce(s_Cannon::setCannonToCoast)).ignoringDisable(true)
        .alongWith(Commands.runOnce(s_LEDs::beWhoYouAre).ignoringDisable(true))
            .withName("SetToCoast"));

    new Trigger(DriverStation::isEnabled)
          .onTrue(Commands.runOnce(s_SwerveDrive::setToBrake)
          .alongWith(Commands.runOnce(s_Cannon::setCannonToBrake)));


    /**
     * HOLD HEADING mode and set a heading
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

    //-> Manual Cannon and Extension triggers    
    xBox.povUp()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRotUp)));
    xBox.povDown()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRotDown)));
    xBox.povRight().debounce(.2)
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manExtend, s_Cannon)));
    xBox.povLeft().debounce(.2)
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRetract, s_Cannon)));

    // combined triggers

    //-> intake trigger
    xBox.rightTrigger(.55).debounce(.1, DebounceType.kFalling)
        .onTrue(Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle))
          .andThen(Commands.runOnce(()-> s_Cannon.setCannonRotation(intakeCannonAngle)))
          .andThen(Commands.runOnce(()-> s_Intake.setIntake(intakeSpeed)))
          .andThen(Commands.waitSeconds(.1).andThen(Commands.waitUntil(s_Intake::checkForCurrentSpike).until(xBox.rightTrigger().negate())))
          .andThen(Commands.runOnce(s_Intake::stopIntake)))
         // .andThen(Commands.waitUntil(s_Intake::haveGamePiece))
         // .andThen(Commands.runOnce(s_Intake::stopIntake)))
        .onFalse(Commands.runOnce(s_Intake::stopIntake));
    
    //-> Outtake trigger
    xBox.leftTrigger(.55).debounce(.1, DebounceType.kFalling)
        .onTrue(Commands.runOnce(()-> s_Intake.setIntake(-intakeSpeed)).alongWith(Commands.runOnce(s_Intake::leaveGamePiece)))
        .onFalse(Commands.runOnce(s_Intake::stopIntake)
          .andThen(Commands.runOnce(()-> s_Cannon.setExtensionInches(1)))
          .andThen(Commands.waitUntil(s_Cannon::extensionErrorWithinRange))
          .andThen(Commands.runOnce(()-> s_Cannon.setCannonRotation(intakeCannonAngle))) //intakecannonangle
          .andThen(Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle))));
    
    //MANUAL INTAKE
    xBox.leftBumper().debounce(.1, DebounceType.kFalling)
        .whileTrue(Commands.runOnce(()-> s_Intake.setIntake(1)))
        .onFalse(Commands.runOnce(s_Intake::stopIntake));
    xBox.rightBumper().debounce(.1, DebounceType.kFalling)
        .whileTrue(Commands.runOnce(()-> s_Intake.setIntake(-1)))
        .onFalse(Commands.runOnce(s_Intake::stopIntake));

    //-> extension + cannonRot to setpoint
    xBox.y()
        .onTrue((Commands.runOnce(()-> {
          s_Cannon.setCannonAngleSides(robotFacing(), NodePosition.getNodePosition(nodeGroupChooser.getSelected(), nodeGridChooser.getSelected()).getCannonAngle());
          // switch ()
        }))
          .andThen(new SequentialCommandGroup(Commands.waitUntil(s_Cannon::cannonErrorWithinRange)),
              Commands.runOnce(()-> s_Cannon.setExtensionInches(NodePosition.getNodePosition(nodeGroupChooser.getSelected(), nodeGridChooser.getSelected()).getExtension())))
          .andThen(Commands.runOnce(() -> {
            if (gamePieceTypeChooser.getSelected() != GamePieceType.CUBE) s_Lid.setLidReference(244);
          })));

    //logic/no controller triggers
    new Trigger(s_Intake::haveGamePiece)
      .onTrue(Commands.runOnce(()-> s_Cannon.setExtensionInches(1))
         .andThen(()-> s_Cannon.setCannonAngleSides(robotFacing(), 140))
          .alongWith(Commands.runOnce(() -> s_LEDs.setMode(LEDs.LEDMode.CHASING)))) // chasing leds because we have a game piece
      .onFalse(Commands.either(
        Commands.runOnce(s_LEDs::bePurple), 
        Commands.runOnce(s_LEDs::beYellow), 
        ()-> (gamePieceTypeChooser.getSelected().equals(GamePieceType.CUBE))));

    new Trigger(() -> robotFacing() != FacingPOI.NOTHING)
      .onTrue(Commands.either(
        Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle))
        .unless(xBox.rightTrigger(.5)),
        Commands.runOnce(() -> s_Cannon.setCannonAngleSides(robotFacing(), 40)).unless(xBox.rightTrigger(.5)) // towards pickup
          .andThen(Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle))), 
           s_Intake::haveGamePiece));
// //TODO: Commented this out because it's not ready  
    // xBox.a().onTrue(new
    // ProxyCommand(()->autoBuilder.followPathGroup(autoPathGroupOnTheFly()))
    // .beforeStarting(new
    // InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(false))));

    new Trigger(gamePieceTypeChooser::didValueChange).and(new Trigger(s_Intake::haveGamePiece).negate())
      .onTrue(Commands.either(
        Commands.runOnce(s_LEDs::bePurple), 
        Commands.runOnce(s_LEDs::beYellow), 
        () -> gamePieceTypeChooser.getSelected().equals(GamePieceType.CUBE)))
      .onTrue(Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle)));
  }

  public void setIntakeParameters() {
    if (pickupLocationChooser.getSelected()==PickupLocation.SHELF) {
   

    } else if (pickupLocationChooser.getSelected()==PickupLocation.CHUTE) {
      
    
    } else if (pickupLocationChooser.getSelected() == PickupLocation.GROUND) { //either facing community or facing HP and picking up from gnd
      //ground
      if (robotFacing() == FacingPOI.HUMAN_PLAYER) {
        switch (gamePieceTypeChooser.getSelected()) {
          case CUBE:
            intakeCannonAngle = -10.0;
            intakeLidAngle = 275.0;
            intakeSpeed = 1.0;
            break;
          case UPRIGHT_CONE:
            intakeCannonAngle = -7.0;
            intakeLidAngle = 206.0;
            intakeSpeed = 1.0;
            break;
          case TIPPED_CONE: // impossible
            intakeCannonAngle = 190.0;
            intakeLidAngle = 244.0;
            intakeSpeed = 1.0;
            break;
          case NOTHING:
            intakeCannonAngle = 85.0;
            break;
        }
      }
      else if (robotFacing() == FacingPOI.COMMUNITY) {
        //facing HPS
        switch (gamePieceTypeChooser.getSelected()) {
          case CUBE:
            intakeCannonAngle = 160.0;
            intakeLidAngle = 275.0;
            intakeSpeed = 1.0;
            break;
          case UPRIGHT_CONE:
            intakeCannonAngle = 175.0;
            intakeLidAngle = 116.0;
            intakeSpeed = -1.0;
            break; 
          case TIPPED_CONE:
            intakeCannonAngle = 190.0;
            intakeLidAngle = 244.0;
            intakeSpeed = 1.0;
            break; 
          case NOTHING: 
            intakeCannonAngle = 85.0;  
            break;
          
          }
        }
    }
  }

  public Command getAutonomousCommand() {
    return autoBuilder.fullAuto(autoSelect.getSelected());
  }
// //TODO: Commented this out because it's not ready
  // public List<PathPlannerTrajectory> autoPathGroupOnTheFly() {
  //   List<PathPlannerTrajectory> PGOTF = autoSelect.getSelected();
  //   PGOTF.add(0,
  //       PathPlanner.generatePath(
  //           new PathConstraints(4, 3),
  //           new PathPoint(s_SwerveDrive.getOdometryPose().getTranslation(), s_SwerveDrive.getOdometryPose().getRotation()),
  //           new PathPoint(new Translation2d(Units.inchesToMeters(nodeGroupChooser.getSelected().xCoord), 
  //             Units.inchesToMeters(nodeGroupChooser.getSelected().yCoord) + Units.inchesToMeters(getYOffset())), 
  //             new Rotation2d(robotFacing() == FacingPOI.COMMUNITY ? 0:180), 
  //             new Rotation2d(robotFacing() == FacingPOI.COMMUNITY ? 0:180))));

  //   return PGOTF;
  // }

  @Log
  public double getYOffset() {
    return NodePosition.getNodePosition(nodeGroupChooser.getSelected(), nodeGridChooser.getSelected()).getYCoord();
  }

  public FacingPOI robotFacing() {
    FacingPOI gyroFacing = FacingPOI.NOTHING;
    if (Math.abs(s_SwerveDrive.getPose().getRotation().getDegrees())<50) // gyro facing community
      gyroFacing = FacingPOI.HUMAN_PLAYER;
    else if (s_SwerveDrive.getPose().getRotation().getDegrees()>130) // gyro facing HP
      gyroFacing = FacingPOI.COMMUNITY;
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
    boolean cannonFacingGyroZero = s_Cannon.getCannonAngleEncoder() < 90.0;

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

  public enum GamePieceType {
    TIPPED_CONE, UPRIGHT_CONE, CUBE, NOTHING;
  }

  public enum NodeDriverStation {
    ONE("driver station one"), TWO("driver station two"), THREE("driver station three");

    public final String dsFriendlyName;

    private NodeDriverStation(String dsFriendlyName) {
      this.dsFriendlyName = dsFriendlyName;
    }
  }

  public static enum ScoringSetPoints {
    HIGH("for high node", 40, 40),
    MID("for mid node", 40, 20),
    LOW("for low/hybrid node", 0, 10),
    UP("straight up", 90, 0),
    ZERO("straight out", 0, 0);

    public final String setPointName;
    public final double cannonAngle;
    public final double cannonExtention;

    ScoringSetPoints(String setPointName, double cannonAngle, double cannonExtention){
      this.setPointName = setPointName;
      this.cannonAngle = cannonAngle;
      this.cannonExtention = cannonExtention;
    }

    public double getTestCannonExtension(){
      return cannonExtention;
    }

    public double getCannonAngle(){
      return cannonAngle;
    }

  }

  public enum PickupLocation {
    GROUND(-14, 10), 
    SHELF(40, 24), 
    CHUTE(4, 24); // change these

    private final double cannonAngle;
    private final double cannonExtension;
   
    private PickupLocation(double cannonAngle, double cannonExtension){this.cannonAngle = cannonAngle; this.cannonExtension = cannonExtension;}
  }
  public double armTestSetPointsAngle(SendableChooser<ScoringSetPoints> armTestSetPoints){
  
    return armTestSetPoints.getSelected().getCannonAngle();
  }


  public Rotation2d calculateHeading(){
    Pose2d roboTranslationX = s_SwerveDrive.getOdometryPose();
    double desiredTranslationX = (Units.inchesToMeters(nodeGroupChooser.getSelected().xCoord));
    double desiredTranslationY = Units.inchesToMeters(nodeGroupChooser.getSelected().yCoord + getYOffset());

    double oOfOA = roboTranslationX.getX() - desiredTranslationX;
    double aOfOA = roboTranslationX.getY() - desiredTranslationY;
    
    return new Rotation2d(-Math.atan(oOfOA/aOfOA));
  }

}
