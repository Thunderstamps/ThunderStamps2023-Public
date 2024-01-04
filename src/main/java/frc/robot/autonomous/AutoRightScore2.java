package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.helpers.*;
import frc.robot.communications.*;
import frc.robot.communications.gamepieces.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.commands.*;
import frc.robot.subsystems.arm.commands.gripper.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;

public class AutoRightScore2 extends AutoBase {

    private static final double interNodeY_in = 22.0; // inches between nodes

    private static final double moveToScoreSpeed_in_sec = 150.0;

    private static final double scoreTweakHeading_rad = Math.toRadians(-2.0);
    
    public AutoRightScore2(
            NetworkTableComms nt,
            SwerveSubsystem swerveSubsystem,
            GripperSubsystem gripperSubsystem,
            JointSubsystem jointSubsystem,
            ExtensionSubsystem extensionSubsystem,
            IntakeMode intakeMode) {

        var autoRightScore1AndPickCommand = new AutoRightScore1AndPick(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode);

        var startPosition_in = autoRightScore1AndPickCommand.getStartPosition_in();
        var communityX_in = startPosition_in.getX() + 22.0; // nominally +27.0
        var communityY_in = startPosition_in.getY() - interNodeY_in + 1.0 + autoRightScore1AndPickCommand.getManualOffsetY_in(); // nominally + 1.0
        var communityPosition_in = Vector2D.FromXY(communityX_in, communityY_in);

        var pickPouncePosition_in = autoRightScore1AndPickCommand.getPickPouncePosition_in();

        addCommands(
            autoRightScore1AndPickCommand,

            // Grip the cone while heading back to score location.
            // This command ends while we're still heading towards the score position and we let inertia take us into position.
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new AutoGripCone(intakeMode, gripperSubsystem),
                    new WaitUntilCommand(swerveSubsystem::isInsideCommunity)
                ),
                new SequentialCommandGroup(
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, pickPouncePosition_in, scoreTweakHeading_rad, moveToScoreSpeed_in_sec, moveToScoreSpeed_in_sec, 10.0),
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, communityPosition_in, scoreTweakHeading_rad, moveToScoreSpeed_in_sec, 0, 3.0),
                    new StopCommand(swerveSubsystem)
                )),

            // Get the cone ready to place, but don't drop it (this is temporary until we can calibrate)
            new ConeGamepieceCommand(nt),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new GripGripperCommand(gripperSubsystem),
                    new ArmToScoreHighCommand(jointSubsystem, extensionSubsystem)
                ), 
                new StopCommand(swerveSubsystem)),

            // Place the cone and retract the extension
            // new ConeGamepieceCommand(nt),
            // new ParallelDeadlineGroup(
            //     new SequentialCommandGroup(
            //         new BeginScoreHighCommand(nt, gripperSubsystem, jointSubsystem, extensionSubsystem),
            //         new RetractExtensionCommand(extensionSubsystem) // EndScoreCommand (below) does this, but there's not enough time before leaving community
            //     ), 
            //     new StopCommand(swerveSubsystem)),
            
            // Finish arm retraction while moving out towards game piece
            // new ParallelDeadlineGroup(
            //     new MoveAbsoluteAndRotateCommand(swerveSubsystem, pickPouncePosition_in, scoreTweakHeading_rad, moveToScoreSpeed_in_sec, moveToScoreSpeed_in_sec, 10.0),
            //     new EndScoreCommand(jointSubsystem, extensionSubsystem)),
            
            new StopCommand(swerveSubsystem)
        );
    }
}
