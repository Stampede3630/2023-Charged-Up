// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;

// import edu.wpi.first.wpilibj.Preferences;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer.GamePieceOrientation;
// import frc.robot.RobotContainer.GamePieceType;
// import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Config;
// import io.github.oblarg.oblog.annotations.Log;
// ///please put current limits on neo 550 20 amps
// //neo current limit 50-60
// public class RotoClawtake extends SubsystemBase implements Loggable {

//   public double rotoMeasure;
//   public double rotoSetPoint = 0;

//   public double clampSetPoint = 9.0;

//   public double clampSqueezeSpeed = 0.4;
//   private boolean haveGamePiece = false;
//   public GamePieceType heldGamePiece = GamePieceType.CUBE;

//   // public Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
//   // public DoubleSolenoid clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
//   public CANSparkMax intakeMotor = new CANSparkMax(54, MotorType.kBrushless);
//   // public CANSparkMax rotoMotor = new CANSparkMax(13, MotorType.kBrushless);
//   public CANSparkMax clampMotor = new CANSparkMax(59, MotorType.kBrushless);

//   // public RelativeEncoder rotoRelativeEncoder = rotoMotor.getEncoder();
//   public RelativeEncoder clampRelativeEncoder = clampMotor.getEncoder();

//   // public AbsoluteEncoder rotoAbsolute =
//   // rotoMotor.getAbsoluteEncoder(Type.kDutyCycle);

//   // public DigitalInput clawSwitch = new DigitalInput(0);

//   // public SparkMaxPIDController rotoMotorPID = rotoMotor.getPIDController();
//   public SparkMaxPIDController clampMotorPID = clampMotor.getPIDController();

//   /** Creates a new Claw. */
//   public RotoClawtake() {

//     // rotoMotor.setIdleMode(IdleMode.kCoast);
//     clampMotor.setIdleMode(IdleMode.kCoast);
//     intakeMotor.setIdleMode(IdleMode.kCoast);

//     // rotoMotor.clearFaults();

//     // rotoMotor.setSmartCurrentLimit(30);
//     clampMotor.setSmartCurrentLimit(30);

//     intakeMotor.setSmartCurrentLimit(60);

//     // rotoRelativeEncoder.setPositionConversionFactor((2 * Math.PI) / 60.0 / 17.6 * 360);
//     // clawRelativeEncoder.setPositionConversionFactor((2 * Math.PI) /
//     // 22.8571428571);

//     clampMotorPID.setP(Preferences.getDouble("ClawKP", 4/0.5));
//     clampMotorPID.setI(Preferences.getDouble("ClawKI", 0.0));
//     clampMotorPID.setD(Preferences.getDouble("ClawPD", 0.0));

//     // rotoMotorPID.setP(Preferences.getDouble("ExtensionKP", 6.0 / Math.PI));
//     // rotoMotorPID.setI(Preferences.getDouble("ExtensionKI", 0.0));
//     // rotoMotorPID.setD(Preferences.getDouble("ExtensionPD", 0.0));

//     // rotoMotor.burnFlash();
//     intakeMotor.burnFlash();
//     clampMotor.burnFlash();
//   }

//   @Override
//   public void periodic() {

//     // rotoMotorPID.setReference(rotoSetPoint, ControlType.kPosition);
//     // This method will be called once per scheduler run
//     if (Preferences.getBoolean("Wanna PID Roto", false)) {
//       // rotoMotorPID.setP(Preferences.getDouble("RotoClawKP", 1.0 / 6.0));
//       // rotoMotorPID.setI(Preferences.getDouble("RotoClawKI", 0.0));
//       // rotoMotorPID.setD(Preferences.getDouble("RotoClawPD", 0.0));
//       Preferences.setBoolean("Wanna PID Roto", false);
//     }

//     haveGamePiece = intakeMotorCurrent() > 40 ? true : haveGamePiece;
//   }

//   public Command initializeClampCommand(){
    
//     return Commands.run(()->clampMotor.set(-0.2))
//     .until(()->clampMotor.getOutputCurrent() > 18.1)
//     .andThen(()->clampRelativeEncoder.setPosition(0))
//     .andThen(()->clampMotor.set(0))
//     .andThen(()->clampMotorPID.setReference(clampSetPoint, ControlType.kPosition))
//     // .unless(limit switch gets a thing)
//     ;

//   }

//   public Command initializeClCommandWithGamePiece(){
//     return Commands.run(()->clampMotor.set(-0.2))
//     .until(()->clampMotor.getOutputCurrent() > 18.1)
//     .andThen(()->clampRelativeEncoder.setPosition(4))
//     .andThen(()->clampMotor.set(0))
//     .andThen(()->clampMotorPID.setReference(clampSetPoint, ControlType.kPosition))
//     // .unless(limit switch gets a thing)
//     ;
//   }

//   public void initializeClamp(){
//     if(clampMotor.getOutputCurrent() < 40){
//       clampMotor.set(-0.1);
//     } else {
//       clampRelativeEncoder.setPosition(0);
//     }
//   }

//   public void stopClawTake() {
//     intakeMotor.set(0);
    
//   }

//   public void stopClawMotor(){
//     clampMotor.set(0);
//   }

//   public void setClawReference(double value) {
//     clampMotorPID.setReference(value, ControlType.kPosition);
//   }
//   public void openClaw() {
//     clampMotorPID.setReference(9.0, ControlType.kPosition);
//     haveGamePiece = false;
//   }

//   public void closeClaw() {
//     // clampMotorPID.setReference(-10, ControlType.kCurrent);
//     clampMotorPID.setReference(1.5, ControlType.kPosition);
//   }

//   // public void setRotoCoast(){
//   //   rotoMotor.setIdleMode(IdleMode.kCoast);
//   //   clampMotor.setIdleMode(IdleMode.kCoast);
//   //   clawTakeMotor.setIdleMode(IdleMode.kCoast);
//   // }

//   // public void setRotoBrake(){
//   //   rotoMotor.setIdleMode(IdleMode.kBrake);
//   //   clampMotor.setIdleMode(IdleMode.kBrake);
//   //   clawTakeMotor.setIdleMode(IdleMode.kBrake);
//   // }

//   public void runClawtake() {
//     // run the rotoclawtake/grab a game piece
//     intakeMotor.set(1);
//   }

//   public void reverseClawtake() {
//     // spit out a game piece/outtake rotoclawtake
//     intakeMotor.set(-1);
//     haveGamePiece = false;
//   }

//   public void setClawReadyPosition(){
//     clampMotorPID.setReference(clampSetPoint, ControlType.kPosition);
//   }

//   public boolean haveGamePiece() {
//     return haveGamePiece;
//   }

//   public void prepareForGamePiece(GamePieceOrientation originalOrientation) {
//     setRotoAngle(originalOrientation.getRotOrientForRoto());
//   }

//   // public void flipRotoClawtake() {
//   //   setRotoAngle((getRotoAngle() + 180) % 360);
//   // }

//   public void faceCommunitySides(double cannonSetPoint) {
//     if (cannonSetPoint < 90)
//       setRotoAngle(180);
//     else
//       setRotoAngle(0);
//   }
//   // @Log
//   // public double getRotoAngle() {
//   //   return rotoRelativeEncoder.getPosition();
//   // }

//   @Config
//   public void setRotoAngle(double input) {
//     rotoSetPoint = input;
//   }

//   @Config
//   public void setHaveGamePiece(boolean input) {
//     haveGamePiece = input;
//   }

//   @Log
//   public boolean getHaveGamePiece() {
//     return haveGamePiece;
//   }

//   @Log
//   public double getClampAngle() {
//     return clampRelativeEncoder.getPosition();
//   }

//   @Config
//   public void setClampSqueezeSpeed(double speed) {
//     clampSqueezeSpeed = speed;
//   }

//   @Log
//   public double clampMotorCurrent() {
//     return clampMotor.getOutputCurrent();
//   }

//   @Log
//   public double intakeMotorCurrent() {
//     return intakeMotor.getOutputCurrent();
//   }

//   @Config(defaultValueBoolean = false)
//   public void initializeClampConfig(boolean input){
//     if(input){
//       initializeClampCommand().schedule();
//     }
//   }

//   @Config
//   public void initializeWithGamePieceConfig(boolean input){
//     if(input){

//     }
//   }

// }