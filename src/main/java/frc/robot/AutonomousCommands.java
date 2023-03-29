package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lid;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.SwerveDrive;

public class AutonomousCommands {
    private final Lid s_Lid = Lid.getInstance();
    private final Cannon s_Cannon = Cannon.getInstance();
    private final Intake s_Intake = Intake.getInstance();
    private final SwerveDrive s_SwerveDrive = SwerveDrive.getInstance();

    private static AutonomousCommands instance;
    public static AutonomousCommands getInstance() {
        if (instance == null)
            instance = new AutonomousCommands();
        return instance;
    }
    public Command autoBalance() {
        return s_SwerveDrive.autoBalanceCommand();
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
                .andThen(s_Cannon.setExtensionWait(NodePosition.NodeGrid.HIGH_LEFT::getExtension))
                .andThen(()->s_Intake.setIntake(-1.0))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(Commands.runOnce(s_Intake::stopIntake))
                .andThen(s_Intake::leaveGamePiece)
                .andThen(s_Cannon.setExtensionWait(() -> 1))
                .andThen(Commands.runOnce(()-> s_Lid.setLid(100)))
                .andThen(Commands.runOnce(()->s_Cannon.setCannonAngle(177.0)));
    }

    public Command autoScoreMidCone() {
        return Commands.runOnce(()->s_Lid.setLid(NodePosition.NodeGrid.MID_LEFT.getNodeLidPositionLidDown()))
                .andThen(Commands.runOnce(()-> s_Cannon.setCannonAngle(NodePosition.NodeGrid.MID_LEFT.getNodeCannonAngleLidDown())))
                .andThen(Commands.waitUntil(s_Cannon::cannonErrorWithinRange))
                .andThen(s_Cannon.setExtensionWait(NodePosition.NodeGrid.MID_LEFT::getExtension))
                .andThen(()->s_Intake.setIntake(-1.0))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(Commands.runOnce(s_Intake::stopIntake))
                .andThen(s_Intake::leaveGamePiece)
                .andThen(s_Cannon.setExtensionWait(() -> 1))
                .andThen(Commands.runOnce(()-> s_Lid.setLid(100)))
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
}