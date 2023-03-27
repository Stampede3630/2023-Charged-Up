// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NodePosition.NodeGrid;
import frc.robot.NodePosition.NodeGroup;
import static frc.robot.Constants.*;

import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.util.ChangeChecker;
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
  private SwerveAutoBuilder otfBuilder;

  // This is just an example event map. It would be better to have a constant,
  // global event map
  // in your code that will be used by all path following commands.

  // This is just an example event map. It would be better to have a constant,
  // global event map

  SendableChooserWrapper<GamePieceType> gamePieceTypeChooser = new SendableChooserWrapper<>();
  SendableChooser<frc.robot.NodePosition.NodeGroup> nodeGroupChooser = new SendableChooser<>();
  SendableChooser<NodeGrid> nodeGridChooser = new SendableChooser<>();
  SendableChooser<PickupLocation> pickupLocationChooser = new SendableChooser<>();
  SendableChooser<List<PathPlannerTrajectory>> autoSelect = new SendableChooser<>();
  PowerDistribution pdh = new PowerDistribution(1,ModuleType.kRev);

  private final SwerveDrive s_SwerveDrive = new SwerveDrive();
  // The robot's subsystems and commands are defined here...
  private final TheCannon s_Cannon = new TheCannon();

  private final LEDs s_LEDs = new LEDs();
  private final Lid s_Lid = Lid.getInstance();
  private final Intake s_Intake = new Intake();
  @Log(tabName = "nodeSelector")
  public double intakeCannonAngle;
  @Log(tabName = "nodeSelector")
  public double intakeLidAngle=Constants.LidConstants.INITIALIZED_ANGLE;
  @Log(tabName = "nodeSelector")
  public double intakeSpeed;

  public boolean facingOverrideButton;

  private double intakeExtensionInches;

  private boolean sniperMode;
  private HashMap<String, Command> eventMap = new HashMap<>();
  private final double[] akitPose = new double[3];
  private final double[] akitInitPose = new double[3];


  // private GamePieceType prev;  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    /*
     * Preferences are cool. they store the values in the roborio flash memory so
     * they don't necessarily get reset to default.
     */
    Preferences.initBoolean("pFieldRelative", DriverConstants.FIELD_RELATIVE);
    Preferences.initBoolean("pAccelInputs", DriverConstants.ACCELERATED_INPUTS);
    Preferences.initDouble("pDriveGovernor", DriverConstants.DRIVE_GOVERNOR);
    Preferences.initBoolean("pOptimizeSteering", SwerveConstants.OPTIMIZESTEERING);
    Preferences.initDouble("pKPRotationController", SwerveConstants.P_ROTATION_CONTROLLER);
    Preferences.initDouble("pKIRotationController", SwerveConstants.D_ROTATION_CONTROLLER);
    Preferences.initDouble("pKDRotationController", SwerveConstants.I_ROTATION_CONTROLLER);
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
        .withWidget("Big Split Button Chooser")
        .withPosition(0, 0)
        .withSize(8, 3);

    Shuffleboard.getTab("nodeSelector")
        .add("Auto Selector", autoSelect)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(17, 0)
        .withSize(3, 1);

    Shuffleboard.getTab("nodeSelector")
      .add("Node Group Chooser", nodeGroupChooser)
      .withWidget("Big Split Button Chooser")
      .withPosition(0, 6)
      .withSize(8, 3);
      
    Shuffleboard.getTab("nodeSelector")
      .add("Node Grid Chooser", nodeGridChooser)
      .withWidget("GridChooserWidget")
      .withProperties(Map.of("Cone","C:\\Users\\Mustangs\\Documents\\evanArtwork\\cone.png",
      "Cube","C:\\Users\\Mustangs\\Documents\\evanArtwork\\cube.png",
      "Hybrid","C:\\Users\\Mustangs\\Documents\\evanArtwork\\hybrid.png",
      "R", 1))
      .withPosition(8, 0)
      .withSize(9, 9);

    Shuffleboard.getTab("nodeSelector")
      .add("Pickup Location", pickupLocationChooser)
      .withWidget("Big Split Button Chooser")
      .withPosition(0, 3)
      .withSize(8, 3);

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("1stBallPickup", new WaitCommand(2));
    eventMap.put("autoScoreHigh", autoScoreHighCube());
    eventMap.put("autoScoreHighCone", autoScoreHighCone());
    eventMap.put("autoIntake", autoIntakeCube());
    eventMap.put("autoBalance", s_SwerveDrive.autoBalanceCommand());
    eventMap.put("autoScoreMidCube", autoScoreMidCube());
    eventMap.put("autoScoreMidCone", autoScoreMidCone());
    eventMap.put("autoScoreLowCube", autoScoreLowCube());

    otfBuilder = new SwerveAutoBuilder(
            s_SwerveDrive::getOdometryPose, // Pose2d supplier
            s_SwerveDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            new PIDConstants(AutoConstants.KP, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
            // PID controllers)
            new PIDConstants(AutoConstants.KP, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
            // controller)
            s_SwerveDrive::setAutoChassisSpeeds, // Module states consumer used to output to the drive subsystem
            eventMap,
            false,
            s_SwerveDrive // The drive subsystem. Used to properly set the requirements of path following
            // commands
    );

    loadPaths();
    configureButtonBindings();

    s_SwerveDrive.setDefaultCommand(
      s_SwerveDrive.joystickDriveCommand(
          () -> xBox.getLeftY() * (sniperMode ? 0.6 : 1),
          () -> xBox.getLeftX() * (sniperMode ? 0.6 : 1),
          () -> xBox.getRightX() * (sniperMode ? 1 : 1))
      .withName("DefaultDrive"));

          Logger.configureLoggingAndConfig(this, false);
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

    /*
     * Trigger Coast/Brake modes when DS is Disabled/Enabled.
     * Trigger runs WHILETRUE for coast mode. Coast Mode method
     * is written to wait for slow speeds before setting to coast
     */

    new Trigger(DriverStation::isDisabled)
        .onTrue(Commands.waitSeconds(5)
            .andThen(Commands.repeatingSequence(Commands.runOnce(s_SwerveDrive::setToCoast)).ignoringDisable(true)
              .alongWith((Commands.runOnce(s_Cannon::setCannonToCoast)).ignoringDisable(true))
              .alongWith(Commands.runOnce(() -> s_LEDs.setMode(LEDMode.RAINBOW)).ignoringDisable(true))
              .alongWith(Commands.runOnce(s_Intake::onDisable)).ignoringDisable(true))
            .withName("SetToCoast"));

    new Trigger(DriverStation::isEnabled)
          .onTrue(Commands.runOnce(s_SwerveDrive::setToBrake)
            .alongWith(Commands.runOnce(() -> {s_LEDs.setChaseColorsSlot(1); s_LEDs.setMode(LEDMode.CHASING);}))
            .alongWith(Commands.runOnce(s_Cannon::setCannonToBrake))
            .alongWith(Commands.runOnce(s_Intake::onEnable)).ignoringDisable(true));


    /*
     * HOLD HEADING mode and set a heading
     */

    xBox.rightStick()
      .debounce(.1)
      .onTrue(Commands.runOnce(() -> sniperMode = true))
      .onFalse(Commands.runOnce(() -> sniperMode = false));
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
    xBox.povRight()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manExtend)));
    xBox.povLeft()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRetract)));
        //MANUAL INTAKE
    xBox.leftBumper().debounce(.1, DebounceType.kFalling)
        .whileTrue(Commands.runOnce(()-> s_Intake.setIntake(1)))
        .onFalse(Commands.runOnce(s_Intake::stopIntake));
    xBox.rightBumper().debounce(.1, DebounceType.kFalling)
        .whileTrue(Commands.runOnce(()-> s_Intake.setIntake(-1)))
        .onFalse(Commands.runOnce(s_Intake::stopIntake));

    // combined triggers

    xBox.b().onTrue(Commands.runOnce(() -> autoPathGroupOnTheFly()).ignoringDisable(true)); // simulate the thing in akit
    //-> intake trigger
    xBox.rightTrigger(.1).debounce(.1, DebounceType.kFalling)
        .onTrue(
          Commands.either(
            s_Cannon.setCannonAngleWait(() -> intakeCannonAngle)
              .andThen(s_Cannon.setExtensionWait(() -> intakeExtensionInches)),
            s_Cannon.setExtensionWait(() -> intakeExtensionInches)
              .andThen(s_Cannon.setCannonAngleWait(() -> intakeCannonAngle)),
            () -> s_Cannon.getExtensionEncoder() < 10) // "dangerous" or not
          .alongWith(Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle)))
          .andThen(Commands.runOnce(()-> s_Intake.setIntake(intakeSpeed)))
          .andThen(s_Intake.waitUntilHaveGamePiece(() -> gamePieceTypeChooser.getSelected() == GamePieceType.CUBE)
            .raceWith(Commands.waitUntil(xBox.rightTrigger(.2).debounce(.2, DebounceType.kFalling).negate())))
          .andThen(Commands.runOnce(s_Intake::stopIntake)))
          //.unless(s_Intake::haveGamePiece))
        .onFalse(Commands.runOnce(s_Intake::stopIntake)
          .andThen(s_Cannon.setExtensionWait(() -> 1))
          .andThen(Commands.runOnce(() -> s_Cannon.setCannonAngleSides(robotFacing(), 90))));
    //-> Outtake trigger
    xBox.leftTrigger(.2).debounce(.15, DebounceType.kBoth)
        .onTrue(Commands.runOnce(()-> s_Intake.setIntake(nodeGridChooser.getSelected().intakeSpeed)).alongWith(Commands.runOnce(s_Intake::leaveGamePiece)))
        .onFalse(Commands.runOnce(s_Intake::stopIntake)
          .andThen(s_Cannon.setExtensionWait(() -> 1))
          .andThen(s_Cannon.setCannonAngleWait(() -> 90))
          .alongWith(Commands.runOnce(()-> s_Lid.setLid(60.0))));
    
    xBox.leftStick().whileTrue((Commands.repeatingSequence(Commands.runOnce(s_SwerveDrive::activateDefensiveStop))));

    //-> extension + cannonRot to setpoint
    xBox.y()
        .onTrue(Commands.runOnce(()-> {
          if (robotFacing()==FacingPOI.COMMUNITY){
            s_Lid.setLidReference(nodeGridChooser.getSelected().getNodeLidPositionLidUp()); 
            s_Cannon.setCannonAngle(nodeGridChooser.getSelected().getNodeCannonAngleLidUp());
          }
          else {
            s_Lid.setLidReference(nodeGridChooser.getSelected().getNodeLidPositionLidDown());
            s_Cannon.setCannonAngle(nodeGridChooser.getSelected().getNodeCannonAngleLidDown());
          }
        }).andThen(Commands.waitUntil(s_Cannon::cannonErrorWithinRange)
            .andThen(Commands.runOnce(()-> s_Cannon.setExtensionReference(nodeGridChooser.getSelected().getExtension()))))
        .alongWith(Commands.runOnce(() -> s_Intake.setIntake(0))));

     //logic/no controller triggers
    new Trigger(s_Intake::haveGamePiece)
      .onTrue(s_Cannon.setExtensionWait(() -> 1)
          .andThen(Commands.runOnce(()-> s_Cannon.setCannonAngleSides(robotFacing(), 90.0)))
        .alongWith(Commands.runOnce(() -> {s_LEDs.setChaseColorsSlot(0); s_LEDs.setMode(LEDs.LEDMode.CHASING);}))) // chasing leds because we have a game piece
      .onFalse(Commands.either(
        Commands.runOnce(s_LEDs::bePurple), 
        Commands.runOnce(s_LEDs::beYellow), 
        ()-> (gamePieceTypeChooser.getSelected().equals(GamePieceType.CUBE))));

    new Trigger(new ChangeChecker<>(this::robotFacing))
      .onTrue(Commands.either(
        Commands.runOnce(()-> s_Cannon.setCannonAngleSides(robotFacing(), 90)),
        Commands.runOnce(() -> s_Cannon.setCannonAngleSides(robotFacing(), 90))
          .alongWith(Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle)))
          .unless(xBox.rightTrigger(.1)),
        s_Intake::haveGamePiece));

    xBox.a().onTrue(new
    ProxyCommand(()-> otfBuilder.followPath(autoPathGroupOnTheFly()))
    .beforeStarting(new InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(false))));

    // xBox.a().onTrue(Commands.runOnce(() -> autoPathGroupOnTheFly()));
    new Trigger(gamePieceTypeChooser::didValueChange).and(() -> !s_Intake.haveGamePiece())
      .onTrue(Commands.either(
        Commands.runOnce(s_LEDs::bePurple), 
        Commands.runOnce(s_LEDs::beYellow), 
        () -> gamePieceTypeChooser.getSelected().equals(GamePieceType.CUBE)))
      .onTrue(Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle)));
  }
  
  public void setIntakeParameters() {
    if (pickupLocationChooser.getSelected()==PickupLocation.SHELF) {
      switch (gamePieceTypeChooser.getSelected()) {
        case CUBE://impossible
          intakeCannonAngle = 180.0;
          intakeLidAngle = 37.5;
          intakeSpeed = -1.0;
          intakeExtensionInches = 0.5; 
          break;
        case TIPPED_CONE: //non existent
        case UPRIGHT_CONE://revise, should be same as tipped
          intakeCannonAngle = 47;
            intakeLidAngle = 35;
            intakeSpeed = 1.0;
            intakeExtensionInches = 30;
            break;
        case NOTHING:
          intakeCannonAngle = 90.0;
          intakeLidAngle = 60.0;
          intakeSpeed = 0.0;
          intakeExtensionInches = 0.0;
          break;
      }

    } else if (pickupLocationChooser.getSelected()==PickupLocation.CHUTE) {     
      switch (gamePieceTypeChooser.getSelected()) {
        case CUBE:
          intakeCannonAngle = 136.5;
          intakeLidAngle = 55.0;
          intakeSpeed = -1.0;
          intakeExtensionInches = 0.5;
          break;
        case TIPPED_CONE:
        case UPRIGHT_CONE://should be same as tipped
//            intakeCannonAngle = 28.0; "community values"
//            intakeLidAngle = 120;
//            intakeSpeed = -1;
//            intakeExtensionInches = 0.5;
            intakeCannonAngle = 177.5;
            intakeLidAngle = 57.5;
            intakeSpeed = 1.0;
            intakeExtensionInches = 0.5;
          break;
        case NOTHING:
          intakeCannonAngle = 90.0;
          intakeLidAngle = 60.0;
          intakeSpeed = 0.0;
          intakeExtensionInches = 0.0;
          break;
      } 
    
    } else if (pickupLocationChooser.getSelected() == PickupLocation.GROUND) { //either facing community or facing HP and picking up from gnd
      //ground
      if (robotFacing() == FacingPOI.COMMUNITY) {
        //facing community, lid up, not able to get tipped cones
        switch (gamePieceTypeChooser.getSelected()) {
          case CUBE://unfavorable cubbe intake!! BOOOOOOO
          intakeCannonAngle = 200.0;
          intakeLidAngle = 60.0;
          intakeSpeed = -1.0;
          intakeExtensionInches = 2.5;
            break;
          case UPRIGHT_CONE://fill in
            intakeCannonAngle = 6.16;
            intakeLidAngle = 46.0;
            intakeSpeed = 1.0;
            intakeExtensionInches = 0.5;
            break;
          case TIPPED_CONE: // no longer impossible
            intakeCannonAngle = -10.99;
            intakeLidAngle = 112.82;
            intakeSpeed = 1.0;
            intakeExtensionInches = 1;
            break;
          case NOTHING:
            intakeCannonAngle = 90.0;
            intakeLidAngle = 60.0;
            intakeSpeed = 0.0;
            intakeExtensionInches = 0.5;
            break;
        }
      }
      else { // gnd hp
        //facing HPS or nothing, lid down
        switch (gamePieceTypeChooser.getSelected()) {
          case CUBE://Caleb 3/14/23
            intakeCannonAngle = 200.0;
            intakeLidAngle = 60.0;
            intakeSpeed = -1.0;
            intakeExtensionInches = 2.5;
            break;
          case UPRIGHT_CONE://Caleb 3/14/23
            intakeCannonAngle = 196;
            intakeLidAngle = 140.0;
            intakeSpeed = 1.0;
            intakeExtensionInches = 1.0;
            break; 
          case TIPPED_CONE://Caleb 3/14/23
          intakeCannonAngle = -10.99;
          intakeLidAngle = 112.82;
          intakeSpeed = 1.0;
          intakeExtensionInches = 1;
            break; 
          case NOTHING: 
            intakeCannonAngle = 90.0;
            intakeLidAngle = 60.0;
            intakeSpeed = 0.0;
            intakeExtensionInches = 0.5;
            break;
          
          }
        }
    }
  }
  
  public Command autoScoreHighCube() { //works if running as NOT first command (for some reason) -ej 3/15
    return Commands.runOnce(()->s_Lid.setLid(100.0))
      .andThen(Commands.runOnce(()-> s_Cannon.setCannonAngle(NodePosition.NodeGrid.HIGH_CENTER.getNodeCannonAngleLidDown())))
      .andThen(Commands.waitUntil(s_Cannon::cannonErrorWithinRange))
      .andThen(Commands.runOnce(()-> s_Cannon.setExtensionReference(23.5)).alongWith(Commands.print("extendo")))
      .andThen(Commands.waitSeconds(0.2))
      .andThen(Commands.waitUntil(s_Cannon::extensionErrorWithinRange))
      .andThen(()->s_Intake.setIntake(0.4))
      .andThen(Commands.waitSeconds(0.5))
      .andThen(Commands.runOnce(s_Intake::stopIntake))
      .andThen(s_Intake::leaveGamePiece)
      .andThen(s_Cannon.setExtensionWait(() -> 1))
      .andThen(Commands.runOnce(()-> s_Lid.setLid(60.0)))
      .andThen(Commands.runOnce(()->s_Cannon.setCannonAngle(120.0)));
      }

  public Command autoScoreHighCone() {
    return Commands.runOnce(()->s_Lid.setLid(NodePosition.NodeGrid.HIGH_LEFT.getNodeLidPositionLidDown()))
      .andThen(Commands.runOnce(()-> s_Cannon.setCannonAngle(NodePosition.NodeGrid.HIGH_LEFT.getNodeCannonAngleLidDown())))
      .andThen(Commands.waitUntil(s_Cannon::cannonErrorWithinRange))
      .andThen(s_Cannon.setExtensionWait(() -> NodePosition.NodeGrid.HIGH_LEFT.getExtension()))
      .andThen(()->s_Intake.setIntake(-1.0))
      .andThen(Commands.waitSeconds(0.5))
      .andThen(Commands.runOnce(s_Intake::stopIntake))
      .andThen(s_Intake::leaveGamePiece)
      .andThen(s_Cannon.setExtensionWait(() -> 1))
      .andThen(Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle)))
      .andThen(Commands.runOnce(()->s_Cannon.setCannonAngle(177.0)));
  }

  public Command autoScoreMidCone() {
    return Commands.runOnce(()->s_Lid.setLid(NodePosition.NodeGrid.MID_LEFT.getNodeLidPositionLidDown()))
      .andThen(Commands.runOnce(()-> s_Cannon.setCannonAngle(NodePosition.NodeGrid.MID_LEFT.getNodeCannonAngleLidDown())))
      .andThen(Commands.waitUntil(s_Cannon::cannonErrorWithinRange))
      .andThen(s_Cannon.setExtensionWait(NodeGrid.MID_LEFT::getExtension))
      .andThen(()->s_Intake.setIntake(-1.0))
      .andThen(Commands.waitSeconds(0.5))
      .andThen(Commands.runOnce(s_Intake::stopIntake))
      .andThen(s_Intake::leaveGamePiece)
      .andThen(s_Cannon.setExtensionWait(() -> 1))
      .andThen(Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle)))
      .andThen(Commands.runOnce(()->s_Cannon.setCannonAngle(177.0)));
  } 

  public Command autoScoreMidCube() { //works -ej 3/15
    return Commands.runOnce(()->s_Lid.setLid(100.0))
    .andThen(Commands.runOnce(()-> s_Cannon.setCannonAngle(38.0)))
    .andThen(Commands.waitUntil(s_Cannon::cannonErrorWithinRange))
    .andThen(s_Cannon.setExtensionWait(() -> 2))
    .andThen(()->s_Intake.setIntake(0.4))
    .andThen(Commands.waitSeconds(0.5))
    .andThen(Commands.runOnce(s_Intake::stopIntake))
    .andThen(s_Intake::leaveGamePiece)
    .andThen(s_Cannon.setExtensionWait(() -> 1))
    .andThen(Commands.runOnce(()-> s_Lid.setLid(100.0)))
    .andThen(Commands.runOnce(()->s_Cannon.setCannonAngle(177.0)));
  }

  public Command autoScoreLowCube() { 
    return Commands.runOnce(()->s_Cannon.setCannonAngle(-7.5))
    .andThen(Commands.runOnce(()->  s_Lid.setLid(80.0)))
    .andThen(Commands.waitUntil(s_Cannon::cannonErrorWithinRange))
    .andThen(s_Cannon.setExtensionWait(() -> 1))
    .andThen(()->s_Intake.setIntake(0.4))
    .andThen(Commands.waitSeconds(0.5))
    .andThen(Commands.runOnce(s_Intake::stopIntake))
    .andThen(s_Intake::leaveGamePiece)
    .andThen(s_Cannon.setExtensionWait(() -> 1))
    .andThen(Commands.runOnce(()-> s_Lid.setLid(100.0)))
    .andThen(Commands.runOnce(()->s_Cannon.setCannonAngle(100.0)));
  }

  public Command autoIntakeCube() { //works -ej 3/15
    return s_Cannon.setCannonAngleWait(() -> 200)
    .andThen(s_Cannon.setExtensionWait(() -> 2.5))
    .andThen(Commands.runOnce(()->s_Lid.setLid(60.0)))
    .andThen(Commands.runOnce(()-> s_Intake.setIntake(-1.0)))
      .andThen(Commands.waitSeconds(.1)
        .andThen(Commands.waitSeconds(2.0)
        .deadlineWith(Commands.waitUntil(s_Intake::checkForCube))))
    .andThen(Commands.runOnce(s_Intake::stopIntake))
    .andThen(Commands.runOnce(()->s_Lid.setLid(100.0)))
    .andThen(s_Cannon.setCannonAngleWait(() -> 90));
  }

  public Command autoIntakeCone() { 
    return s_Cannon.setCannonAngleWait(() -> 193)
    .andThen(s_Cannon.setExtensionWait(() -> 1))
    .andThen(Commands.runOnce(()->s_Lid.setLid(140.0)))
    .andThen(Commands.runOnce(()-> s_Intake.setIntake(1.0))
      .andThen(Commands.waitSeconds(.1)
        .andThen(Commands.waitSeconds(2.0)))
        .deadlineWith(s_Intake.waitUntilHaveGamePiece(() -> false)))
    .andThen(Commands.runOnce(s_Intake::stopIntake))
    .andThen(Commands.runOnce(()->s_Lid.setLid(100.0)));
  }

  private void loadPaths() {
    autoBuilder = new SwerveAutoBuilder(
            s_SwerveDrive::getOdometryPose, // Pose2d supplier
            s_SwerveDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            new PIDConstants(AutoConstants.KP, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
            // PID controllers)
            new PIDConstants(AutoConstants.KP, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
            // controller)
            s_SwerveDrive::setAutoChassisSpeeds, // Module states consumer used to output to the drive subsystem
            eventMap,
            true,
            s_SwerveDrive // The drive subsystem. Used to properly set the requirements of path following
            // commands
    );
    eventMap.put("1stBallPickup", new WaitCommand(2));
    eventMap.put("autoScoreHigh", autoScoreHighCube());
    eventMap.put("autoScoreHighCone", autoScoreHighCone());
    eventMap.put("autoIntake", autoIntakeCube());
    eventMap.put("autoIntakeCone", autoIntakeCone());
    eventMap.put("autoBalance", s_SwerveDrive.autoBalanceCommand());
    eventMap.put("autoScoreMidCube", autoScoreMidCube());
    eventMap.put("autoScoreMidCone", autoScoreMidCone());
    eventMap.put("autoScoreLowCube", autoScoreLowCube());

    // load autos completely dynamically -- any autos in pathplanner folder will be added to selector
    List<File> files = List.of(
            Objects.requireNonNull(new File(Filesystem.getDeployDirectory(), "pathplanner")
                    .listFiles((dir, name) -> name.endsWith(".path"))));
    for (File file : files) {
      String pathName = file.getName().split("\\.")[0];
      autoSelect.addOption(pathName, PathPlanner.loadPathGroup(pathName,
        new PathConstraints(3.25, 2)));
    }
    autoSelect.setDefaultOption(files.get(0).getName().split("\\.")[0], PathPlanner.loadPathGroup(files.get(0).getName().split("\\.")[0], new PathConstraints(3.25, 2)));
  }

  public Command getAutonomousCommand() {
    return autoBuilder.fullAuto(autoSelect.getSelected());
  }
  
  public PathPlannerTrajectory autoPathGroupOnTheFly() {
    double x = Units.inchesToMeters(nodeGroupChooser.getSelected().xCoord);
    double y = Units.inchesToMeters(NodePosition.getNodePosition(nodeGroupChooser.getSelected(),nodeGridChooser.getSelected()).getYCoord());
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      NodeGroup nodeGroup;
      NodeGrid nodeGrid;
      switch (nodeGroupChooser.getSelected()) { // flip things when on red
        case LEFT:
          nodeGroup = NodeGroup.RIGHT; break;
        case RIGHT:
          nodeGroup = NodeGroup.LEFT; break;
        default:
          nodeGroup = nodeGroupChooser.getSelected(); break;
      }
      switch (nodeGridChooser.getSelected()) { // flip things when on red
        case MID_LEFT:
          nodeGrid = NodeGrid.MID_RIGHT; break;
        case LOW_LEFT:
          nodeGrid = NodeGrid.LOW_RIGHT; break;
        case HIGH_LEFT:
          nodeGrid = NodeGrid.HIGH_RIGHT; break;
        case MID_RIGHT:
          nodeGrid = NodeGrid.MID_LEFT; break;
        case LOW_RIGHT:
          nodeGrid = NodeGrid.LOW_LEFT; break;
        case HIGH_RIGHT:
          nodeGrid = NodeGrid.HIGH_LEFT; break;
        default:
          nodeGrid = nodeGridChooser.getSelected();
          break;
      }
      y = Units.inchesToMeters(NodePosition.getNodePosition(nodeGroup,nodeGrid).getYCoord());
      y = 8.01367968 - y;
    }
    Pose2d initialPose = s_SwerveDrive.getOdometryPose();
    PathPlannerTrajectory PGOTF = PathPlanner.generatePath(
        new PathConstraints(0.5, 0.5),
        new PathPoint(initialPose.getTranslation(), calculateHeading(x,y), initialPose.getRotation()),
        new PathPoint(new Translation2d(x, y), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
      );
    akitInitPose[0] = PGOTF.getInitialPose().getX();
    akitInitPose[1] = PGOTF.getInitialPose().getY();
    akitInitPose[2] = PGOTF.getInitialPose().getRotation().getRadians();
    akitPose[0] = PGOTF.getEndState().poseMeters.getX();
    akitPose[1] = PGOTF.getEndState().poseMeters.getY();
    akitPose[2] = PGOTF.getEndState().poseMeters.getRotation().getRadians();
    SmartDashboard.putNumberArray("pgotf", akitPose);
    SmartDashboard.putNumberArray("pgotfInit", akitInitPose);
    return PGOTF;
  }

  public double getYOffset() {
    return NodePosition.getNodePosition(nodeGroupChooser.getSelected(), nodeGridChooser.getSelected()).getYCoord();
  }

  public FacingPOI robotFacing() {
    FacingPOI gyroFacing = FacingPOI.NOTHING;
    if (Math.abs(s_SwerveDrive.getPose().getRotation().getDegrees())<75) // gyro facing community
      gyroFacing = FacingPOI.HUMAN_PLAYER;
    else if (Math.abs(s_SwerveDrive.getPose().getRotation().getDegrees())>105) // gyro facing HP
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
        return FacingPOI.HUMAN_PLAYER;
      else
        return FacingPOI.COMMUNITY;
    else // gyro must be facing HP
      if (cannonFacingGyroZero)
        return FacingPOI.COMMUNITY;
      else
        return FacingPOI.HUMAN_PLAYER;
  }

  public enum FacingPOI {
    COMMUNITY, HUMAN_PLAYER, NOTHING
  }

  public enum GamePieceType {
    TIPPED_CONE, UPRIGHT_CONE, CUBE, NOTHING
  }


  public enum PickupLocation {
    GROUND, SHELF, CHUTE
  }

  public Rotation2d calculateHeading(double desiredX, double desiredY){
    Pose2d roboTranslation = s_SwerveDrive.getOdometryPose();
    double adj = desiredX - roboTranslation.getX();
    double opp = desiredY - roboTranslation.getY();

    if (adj >= 0) // quad I or IV
      return new Rotation2d(Math.atan(opp/adj));
    else if (opp > 0) // quad II
      return new Rotation2d(Math.atan(opp/adj)+Math.PI);
    else // opp < 0, quad III
      return new Rotation2d(-(Math.PI)+Math.atan(opp/adj));
  }
  

  @Config
  public void setCannonOffset(double input){
    s_Cannon.cannonReference += input;
  }

}
