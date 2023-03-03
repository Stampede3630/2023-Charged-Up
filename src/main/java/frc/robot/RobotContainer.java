// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.opencv.core.Scalar;

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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
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
import frc.robot.subsystems.Lid;
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
  SendableChooser<GamePieceType> gamePieceTypeChooser = new SendableChooser<>();
  SendableChooser<frc.robot.NodePosition.NodeGroup> nodeGroupChooser = new SendableChooser<>();
  SendableChooser<NodeGrid> nodeGridChooser = new SendableChooser<>();
  SendableChooser<PickupLocation> pickupLocationChooser = new SendableChooser<>();

  @Log
  private final SwerveDrive s_SwerveDrive = new SwerveDrive();
  // The robot's subsystems and commands are defined here...
  private final TheCannon s_Cannon = new TheCannon();

  private final LEDs s_LEDs = new LEDs();
  private final Lid s_Lid = new Lid();
  private final Intake s_Intake = new Intake();
  
  public double intakeCannonAngle;
  public double intakeLidAngle;
  public double intakeSpeed;
  public GamePieceType previousGamePieceOrientation;
  public GamePieceType gamePieceTypeLed;
  public boolean facingOverrideButton;
  // private GamePieceType prev;  
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
    Preferences.initDouble("LidKP", 1 / 30.0);
    Preferences.initDouble("LidKI", 0.0);
    Preferences.initDouble("LidKD", 0.0);
    Preferences.initBoolean("Wanna PID Roto", false);
    Preferences.initBoolean("Wanna PID Cannon", false);
    Preferences.initBoolean("Wanna PID Lid", false);



    autoPathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("chargeSimpleRed",
        new PathConstraints(.5, .5));

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


    // Shuffleboard.getTab("nodeSelector")
    //     .add("Node Driver Station", nodeDriverStation)
    //     .withWidget(BuiltInWidgets.kComboBoxChooser);
      
    // Shuffleboard.getTab("nodeSelector")
    //     .add("Arm Point Select", armTestSetPoints)
    //     .withWidget(BuiltInWidgets.kComboBoxChooser);

    Shuffleboard.getTab("nodeSelector")
      .add("Node Group Chooser", nodeGroupChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser);
      
    Shuffleboard.getTab("nodeSelector")
      .add("Node Grid Chooser", nodeGridChooser)
      .withWidget("GridChooserWidget")
      .withProperties(Map.of("Cone Image","C:\\Users\\Mustangs\\Documents\\evanArtwork\\cone.png",
      "Cube Image","C:\\Users\\Mustangs\\Documents\\evanArtwork\\cube.png",
      "Hybrid Image","C:\\Users\\Mustangs\\Documents\\evanArtwork\\hybrid.png",
      "R", 1));

    Shuffleboard.getTab("nodeSelector")
      .add("Pickup Location", pickupLocationChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser);

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
        .whileTrue((Commands.runOnce(s_Cannon::setCannonToCoast))
        // .alongWith(Commands.runOnce(s_Claw::setRotoCoast))
        .alongWith(Commands.runOnce(s_LEDs::beWhoYouAre).ignoringDisable(true))
            .withName("SetToCoast"));
    new Trigger(DriverStation::isEnabled)
          .onTrue(Commands.runOnce(s_SwerveDrive::setToBrake)
          .alongWith(Commands.runOnce(s_Cannon::setCannonToBrake)));
          // .alongWith(Commands.runOnce(s_Claw::setRotoBrake))

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

    //testing triggers    
    xBox.povUp()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRotUp)));
    xBox.povDown()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRotDown)));
    xBox.povRight()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manExtend, s_Cannon)));
    xBox.povLeft()
        .whileTrue(new RepeatCommand(Commands.runOnce(s_Cannon::manRetract, s_Cannon)));

    //combined triggers

    //-> intake trigger
    // xBox.rightTrigger(.55).debounce(.1, DebounceType.kFalling)
    //     .onTrue(Commands.runOnce(s_Claw::runClawtake)
    //       .alongWith(Commands.runOnce(() -> s_Cannon.setCannonAngleSides(robotFacing(), pickupLocationChooser.getSelected().cannonAngle))
    //       .alongWith(Commands.waitUntil(s_Cannon::cannonErrorWithinRange)
    //         .andThen(() -> s_Cannon.setExtensionInches(pickupLocationChooser.getSelected().cannonExtension)))
    //         )) //.alongWith(Commands.runOnce(s_Claw::closeClaw))
    //     .onFalse(Commands.runOnce(s_Claw::stopClawTake)
    //       .andThen(Commands.runOnce(() -> s_Cannon.setExtensionInches(4)))
    //         .andThen(Commands.either(
    //         Commands.runOnce(() -> s_Claw.faceCommunitySides(s_Cannon.setCannonAngleSides(robotFacing(), 140)))
    //         .unless(xBox.rightTrigger(.5)),
    //         Commands.runOnce(() -> s_Cannon.setCannonAngleSides(robotFacing(), 40)).unless(xBox.rightTrigger(.5)) // towards pickup
    //           .andThen(Commands.runOnce(() -> s_Claw.prepareForGamePiece(gamePieceTypeChooser.getSelected()))), 
    //             s_Claw::haveGamePiece)));
    xBox.rightTrigger(.55).debounce(.1, DebounceType.kFalling)
        .onTrue(Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle))
          .andThen(Commands.runOnce(()-> s_Cannon.setCannonRotation(intakeCannonAngle)))
          .andThen(Commands.runOnce(()-> s_Intake.setIntake(intakeSpeed))))
         // .andThen(Commands.waitUntil(s_Intake::haveGamePiece))
         // .andThen(Commands.runOnce(s_Intake::stopIntake)))
        .onFalse(Commands.runOnce(s_Intake::stopIntake));
    
    xBox.leftTrigger(.55).debounce(.1, DebounceType.kFalling)
        .onTrue(Commands.runOnce(()-> s_Intake.setIntake(-intakeSpeed)).alongWith(Commands.runOnce(s_Intake::leaveGamePiece)))
        .onFalse(Commands.runOnce(s_Intake::stopIntake)
          .andThen(Commands.runOnce(()-> s_Cannon.setExtensionInches(0)))
          .andThen(Commands.waitUntil(s_Cannon::extensionErrorWithinRange))
          .andThen(Commands.runOnce(()-> s_Cannon.setCannonRotation(intakeCannonAngle))) //intakecannonangle
          .andThen(Commands.runOnce(()-> s_Lid.setLid(intakeLidAngle))));

    xBox.rightBumper()
        .onTrue(Commands.runOnce(()-> s_Intake.setIntake(1)))
        .onFalse(Commands.runOnce(s_Intake::stopIntake));
    xBox.rightBumper()
        .onTrue(Commands.runOnce(()-> s_Intake.setIntake(-1)))
        .onFalse(Commands.runOnce(s_Intake::stopIntake));
    //-> outtake trigger
    // xBox.leftTrigger(.55).debounce(.1, DebounceType.kFalling)
    //     .onTrue((Commands.runOnce(s_Claw::reverseClawtake)))
    //     .onFalse(Commands.runOnce(s_Claw::stopClawTake)
    //       .andThen(Commands.runOnce(()-> s_Cannon.setExtensionInches(5.0))
    //         .andThen(Commands.waitUntil(s_Cannon::extensionErrorWithinRange)
    //           .andThen(Commands.runOnce(()-> s_Cannon.setCannonAngleSides(robotFacing(), 150))))));

    //-> extension + cannonRot to setpoint
    xBox.y()
        .onTrue((Commands.runOnce(()-> s_Cannon.setCannonAngleSides(robotFacing(), NodePosition.getNodePosition(nodeGroupChooser.getSelected(), nodeGridChooser.getSelected()).getCannonAngle())))
          .andThen(new SequentialCommandGroup(Commands.waitUntil(s_Cannon::cannonErrorWithinRange)),
              Commands.runOnce(()-> s_Cannon.setExtensionInches(NodePosition.getNodePosition(nodeGroupChooser.getSelected(), nodeGridChooser.getSelected()).getExtension()))));

    //single object triggers

    // //->claw
    // xBox.rightBumper().debounce(.1, DebounceType.kFalling)
    //     .onTrue(Commands.repeatingSequence(Commands.runOnce(() -> s_Claw.setClawReference(s_Claw.getClampAngle()+5))))
    //     .onFalse(Commands.runOnce(s_Claw::stopClawMotor));
    // xBox.leftBumper().debounce(.1, DebounceType.kFalling)
    //     .onTrue(Commands.repeatingSequence(Commands.runOnce(() -> s_Claw.setClawReference(s_Claw.getClampAngle()-5))))
    //     .onFalse(Commands.runOnce(s_Claw::stopClawMotor));

    // xBox.x().debounce(.1)
    //     .onTrue(Commands.repeatingSequence(Commands.runOnce(() -> s_Claw.setRotoAngle(s_Claw.getRotoAngle() + 5))));      
        
    // xBox.b().debounce(.1)
        // .onTrue(Commands.repeatingSequence(Commands.runOnce(() -> s_Claw.setRotoAngle(s_Claw.getRotoAngle() - 5))));

    //logic/no controller triggers
    new Trigger(s_Intake::haveGamePiece)
      .onTrue(Commands.runOnce(()-> s_Cannon.setExtensionInches(1.97985 + 1))
         .andThen(()-> s_Cannon.setCannonAngleSides(robotFacing(), 140))
          .andThen(() -> s_LEDs.rainbow = true))
      .onFalse(Commands.either(
        Commands.runOnce(s_LEDs::bePurple), 
        Commands.runOnce(s_LEDs::beYellow), 
        ()-> (gamePieceTypeChooser.getSelected().equals(GamePieceType.CUBE))));

    new Trigger(() -> robotFacing() != FacingPOI.NOTHING)
      .onTrue(Commands.either(
        Commands.runOnce(this::setLid)
        .unless(xBox.rightTrigger(.5)),
        Commands.runOnce(() -> s_Cannon.setCannonAngleSides(robotFacing(), 40)).unless(xBox.rightTrigger(.5)) // towards pickup
          .andThen(Commands.runOnce(this::setLid)), 
           s_Intake::haveGamePiece));
  
    xBox.a().onTrue(new
    ProxyCommand(()->autoBuilder.followPathGroup(autoPathGroupOnTheFly()))
    .beforeStarting(new
    InstantCommand(()->s_SwerveDrive.setHoldHeadingFlag(false))));

    new Trigger(()->listenForOrientationChange()).and(s_Intake::haveGamePiece).negate()
      .onTrue(Commands.either(
        Commands.runOnce(s_LEDs::bePurple), 
        Commands.runOnce(s_LEDs::beYellow), 
        () -> previousGamePieceOrientation.equals(GamePieceType.CUBE)))
      .onTrue(Commands.runOnce(this::setLid));

  }

  public void setIntakeParameters() {
    if (robotFacing() == FacingPOI.HUMAN_PLAYER && pickupLocationChooser.getSelected()==PickupLocation.SHELF) {
   

    } else if (robotFacing() == FacingPOI.HUMAN_PLAYER && pickupLocationChooser.getSelected()==PickupLocation.CHUTE) {
      
    
    } else {
      //ground
    if (robotFacing() == FacingPOI.COMMUNITY && !facingOverrideButton) {
      switch (gamePieceTypeChooser.getSelected()) {
        case CUBE:
          intakeCannonAngle = -10.0;
          intakeLidAngle = 275.0;
          intakeSpeed = -1.0;
          break;
        case UPRIGHT_CONE:
          intakeCannonAngle = -7.0;
          intakeLidAngle = 206.0;
          intakeSpeed = 1.0;
          break;
        case TIPPED_CONE:
          intakeCannonAngle = 190.0;
          intakeLidAngle = 244.0;
          intakeSpeed = -1.0;
          break;
        case NOTHING:
          intakeCannonAngle = 85.0;
          break;
      
      }
    } if (robotFacing() == FacingPOI.COMMUNITY && facingOverrideButton) {
        //facing HPS
        switch (gamePieceTypeChooser.getSelected()) {
          case CUBE:
          intakeCannonAngle = -10.0;
          intakeLidAngle = 275.0;
          intakeSpeed = -1.0;
            break;
          case UPRIGHT_CONE:
            intakeCannonAngle = 175.0;
            intakeLidAngle = 116.0;
            intakeSpeed = -1.0;
            break; 
          case TIPPED_CONE:
            intakeCannonAngle = 190.0;
            intakeLidAngle = 244.0;
            intakeSpeed = -1.0;
            break; 
          case NOTHING: 
            intakeCannonAngle = 85.0;
            break; 
       
      }
    } if (robotFacing() == FacingPOI.HUMAN_PLAYER && !facingOverrideButton) {
      //facing HPS
      switch (gamePieceTypeChooser.getSelected()) {
        case CUBE:
        intakeCannonAngle = 160.0;
          intakeLidAngle = 275.0;
          intakeSpeed = -1.0;
          break;
        case UPRIGHT_CONE:
          intakeCannonAngle = 175.0;
          intakeLidAngle = 116.0;
          intakeSpeed = -1.0;
          break; 
        case TIPPED_CONE:
          intakeCannonAngle = 190.0;
          intakeLidAngle = 244.0;
          intakeSpeed = -1.0;
          break; 
        case NOTHING: 
          intakeCannonAngle = 85.0;  
          break;
         
        } 
      
      if (robotFacing() == FacingPOI.HUMAN_PLAYER && facingOverrideButton) {
        //facing HPS
        switch (gamePieceTypeChooser.getSelected()) {
          case CUBE:
            intakeCannonAngle = -10.0;
            intakeLidAngle = 275.0;
            intakeSpeed = -1.0;
            break;
          case UPRIGHT_CONE:
            intakeCannonAngle = -7.0;
            intakeLidAngle = 206.0;
            intakeSpeed = 1.0;
            break;
          case TIPPED_CONE:
            intakeCannonAngle = 190.0;
            intakeLidAngle = 244.0;
            intakeSpeed = -1.0;
            break;
          case NOTHING:
            intakeCannonAngle = 85.0;  
            break;
        
        }

    } else {
      //should never happen
      switch (gamePieceTypeChooser.getSelected()) {
        case CUBE:
        intakeCannonAngle = -10.0;
          intakeLidAngle = 275.0;
          intakeSpeed = -1.0;
          break;
        case UPRIGHT_CONE:
          intakeCannonAngle = 175.0;
          intakeLidAngle = 116.0;
          intakeSpeed = -1.0;
          break; 
        case TIPPED_CONE:
          intakeCannonAngle = 190.0;
          intakeLidAngle = 244.0;
          intakeSpeed = -1.0;
          break; 
        case NOTHING: 
          intakeCannonAngle = 85.0;
          break; 
    }
  }

  }

    
  }
}

  private void setLid() {
    if (robotFacing() == FacingPOI.HUMAN_PLAYER) {
      switch (pickupLocationChooser.getSelected()) {
        case GROUND: 
        case SHELF:
          s_Lid.setLipOut();
          break;
        case CHUTE:
          s_Lid.setLipIn();
      }
    } else if (robotFacing() == FacingPOI.COMMUNITY) {
      switch (pickupLocationChooser.getSelected()) {
        case SHELF: break;
        case CHUTE: s_Lid.setLipIn();
        case GROUND:
          switch (gamePieceTypeChooser.getSelected()) {
            case TIPPED_CONE: break;
            case UPRIGHT_CONE: case CUBE: s_Lid.setLipIn();
            case NOTHING: break;
          }
      }
    }
  }

  public Command getAutonomousCommand() {
    return autoBuilder.fullAuto(autoPathGroup).withName("chargeSimpleRed");
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

  // public enum GamePieceOrientation {
  //   TIPPED("|>", 90, GamePieceType.CONE),
  //   UPRIGHT("/\\", 0, GamePieceType.CONE),
  //   CUBE("口", 90, GamePieceType.CUBE);

  //   private String friendlyName;
  //   private double rotOrientationAngle;
  //   private GamePieceType gamePieceType;

  //   public String getFriendlyName() {
  //     return friendlyName;
  //   }

  //   public double getRotOrientForRoto() {
  //     return rotOrientationAngle;
  //   }

  //   public GamePieceType getGamePieceType() {
  //     return gamePieceType;
  //   }

  //   private GamePieceOrientation(String friendlyName, double rotOrientationAngle, GamePieceType gamePieceType) {
  //     this.friendlyName = friendlyName;
  //     this.rotOrientationAngle = rotOrientationAngle;
  //     this.gamePieceType = gamePieceType;
  //   }

  // }

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

  public enum PickupLocation {
    GROUND(-14, 10), 
    SHELF(40, 24), 
    CHUTE(4, 24); // change these

    private final double cannonAngle;
    private final double cannonExtension;
   
    private PickupLocation(double cannonAngle, double cannonExtension){this.cannonAngle = cannonAngle; this.cannonExtension = cannonExtension;}
  }
  public double armTestSetPointsAngle(SendableChooser<ArmTestSetPoints> armTestSetPoints){
  
    return armTestSetPoints.getSelected().getTestCannonAngle();
  }

  public boolean listenForOrientationChange(){

    previousGamePieceOrientation = gamePieceTypeChooser.getSelected();

    if (gamePieceTypeChooser.getSelected() != previousGamePieceOrientation) {
      previousGamePieceOrientation = gamePieceTypeChooser.getSelected();

      return true;
    } else{

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

  // public boolean listenForGamePieceLED(){
  //
  //   gamePieceTypeLed = gamePieceTypeChooser.getSelected().getGamePieceType();
  //
  //   if (gamePieceTypeChooser.getSelected().getGamePieceType() != gamePieceTypeLed) {
  //     gamePieceTypeLed = gamePieceTypeChooser.getSelected().getGamePieceType();
  //     System.out.println("a thing happened");
  //     return true;
  //   } else{
  //     System.out.println("a thing did not happen");
  //     return false;
  //   }
  // }

  public boolean isCone(){
    return gamePieceTypeChooser.getSelected() == GamePieceType.TIPPED_CONE || gamePieceTypeChooser.getSelected() == GamePieceType.UPRIGHT_CONE;

  }

  public boolean isCube(){
    return gamePieceTypeChooser.getSelected().equals(GamePieceType.CUBE);

  }

  public boolean isNoThing(){
    return gamePieceTypeChooser.getSelected().equals(GamePieceType.NOTHING);
    //Not necessary because isNoThing is never true 6:13 PM Friday, February 24 (=^-w-^=), :3, UwU, >wO
  }

  // public boolean beDesiredPieceColorR(){
  //   if (gamePieceType == gamePieceTypeChooser.getSelected().getGamePieceType().CONE){
  //     return false;
  //   } else {
  //     return true;
  //   }
  // }
  
  @Config
  public void facingOverrideButton(boolean input){
    facingOverrideButton = input;
  }

}
