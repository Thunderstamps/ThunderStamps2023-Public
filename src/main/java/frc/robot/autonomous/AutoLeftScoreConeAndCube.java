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

public class AutoLeftScoreConeAndCube extends AutoBase {

    private static final double interNodeY_in = 22.0; // inches between nodes

    private static final double moveToScoreSpeed_in_sec = 150.0;

    private static final double scoreTweakHeading_rad = Math.toRadians(0.0);
    
    public AutoLeftScoreConeAndCube(
            NetworkTableComms nt,
            SwerveSubsystem swerveSubsystem,
            GripperSubsystem gripperSubsystem,
            JointSubsystem jointSubsystem,
            ExtensionSubsystem extensionSubsystem,
            IntakeMode intakeMode) {

        var autoLeftScoreConeAndPickCommand = new AutoLeftScoreConeAndPick(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode);

        var startPosition_in = autoLeftScoreConeAndPickCommand.getStartPosition_in();
        var cubePounceX_in = startPosition_in.getX() + 20.0;
        var cubePounceY_in = startPosition_in.getY() - interNodeY_in;
        var cubePouncePosition_in = Vector2D.FromXY(cubePounceX_in, cubePounceY_in);
        
        var clearPosition_in = Vector2D.FromXY(cubePounceX_in + 30.0, cubePounceY_in + 12.0);

        var pickPouncePosition_in = autoLeftScoreConeAndPickCommand.getPickPouncePosition_in();
        var pickPouncePosition2_in = 
            Vector2D.FromXY(pickPouncePosition_in.getX(), pickPouncePosition_in.getY() - 24.0);

        var cubeDropPosition_in = Vector2D.FromXY(
            startPosition_in.getX(), 
            startPosition_in.getY() - interNodeY_in);

        addCommands(
            autoLeftScoreConeAndPickCommand,

            // Grip the cube while heading back to score location.
            new ParallelDeadlineGroup(
                new AutoGripCone(intakeMode, gripperSubsystem), 
                new StopCommand(swerveSubsystem)),

            // This command ends while we're still heading towards the score position and we let inertia take us into position.
            new CubeGamepieceCommand(nt),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(swerveSubsystem::isInsideCommunity),
                    new ArmToScoreHighCommand(jointSubsystem, extensionSubsystem),
                    new WaitCommand(1.0),
                    new WaitUntilCommand(nt::isTargetFound),
                    new ReleaseGripperCommand(gripperSubsystem),
                    new WaitCommand(0.25)
                ),
                new SequentialCommandGroup(
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, pickPouncePosition_in, scoreTweakHeading_rad, moveToScoreSpeed_in_sec, moveToScoreSpeed_in_sec, 10.0),
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, clearPosition_in, scoreTweakHeading_rad, moveToScoreSpeed_in_sec, 60, 25.0),
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, cubePouncePosition_in, scoreTweakHeading_rad, moveToScoreSpeed_in_sec, 36, 12.0),
                    new AutolineupCommand(swerveSubsystem)
                )),
            
            // Finish arm retraction while moving out towards game piece
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new SetRobotFieldPositionCommand(swerveSubsystem, cubeDropPosition_in.getX(), cubeDropPosition_in.getY()),
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, cubePouncePosition_in, 0, moveToScoreSpeed_in_sec, moveToScoreSpeed_in_sec, 5.0),
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, clearPosition_in, 0, moveToScoreSpeed_in_sec, moveToScoreSpeed_in_sec, 8.0),
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, pickPouncePosition_in, 0, moveToScoreSpeed_in_sec, moveToScoreSpeed_in_sec, 15.0),
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, pickPouncePosition2_in, Math.toRadians(-45.0), moveToScoreSpeed_in_sec, moveToScoreSpeed_in_sec, 5.0)
                ),
                new EndScoreCommand(jointSubsystem, extensionSubsystem)),
            
            new StopCommand(swerveSubsystem)
        );
    }
}
