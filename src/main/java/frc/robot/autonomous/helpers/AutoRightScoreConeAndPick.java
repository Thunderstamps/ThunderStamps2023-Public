package frc.robot.autonomous.helpers;

import java.util.ArrayList;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.*;
import frc.robot.communications.gamepieces.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.commands.gripper.GripGripperCommand;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.commands.SetIntakeMode;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;

public class AutoRightScoreConeAndPick extends AutoBase {

    // Ontario Champs:
    // Red:  -10.0
    // Blue:  -6.0
    private static final double manualOffsetY_in = -10.0; // was 0.0 at Western, -12.0 at windsor, not used except in practice

    private static final double startX_in = 56.0;
    private static final double startY_in = 19.0; // 8 grids to the right of the left start Y position, 195.0 - 22.0 * 8

    private static final double noseOutOfGridX_in = 9.0; // how far to move to get nose out of grid

    private static final double pickPounceX_in = 215.0; // nominally 220
    private static final double pickPounceY_in = 36.0;  // 12 feet to the right of the left pick pounce position, 180.0 - 144.0

    private static final double pickX_in = 305.0;
    private static final double pickY_in = pickPounceY_in;

    private static final Vector2D startPosition_in = Vector2D.FromXY(startX_in, startY_in);

    private static final Vector2D mid1APosition_in = Vector2D.FromXY(startX_in + noseOutOfGridX_in, startY_in);
    private static final Vector2D pickPouncePosition_in = Vector2D.FromXY(pickPounceX_in, pickPounceY_in + manualOffsetY_in);
    private static final Vector2D pickPosition_in = Vector2D.FromXY(pickX_in, pickY_in + manualOffsetY_in);

    public AutoRightScoreConeAndPick(
            NetworkTableComms nt,
            SwerveSubsystem swerveSubsystem,
            GripperSubsystem gripperSubsystem,
            JointSubsystem jointSubsystem,
            ExtensionSubsystem extensionSubsystem,
            IntakeMode intakeMode) {

        var pickPounceTrajectory = makeTrajectoryToPickPounce();

        addCommands(
            new SetRobotFieldPositionCommand(swerveSubsystem, startPosition_in.getX(), startPosition_in.getY()),

            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new ConeGamepieceCommand(nt),
                    new GripGripperCommand(gripperSubsystem),
                    new WaitCommand(0.25),
                    new BeginScoreHighCommand(nt, gripperSubsystem, jointSubsystem, extensionSubsystem, 1.0)
                ), 
                new StopCommand(swerveSubsystem)),

            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(swerveSubsystem, pickPounceTrajectory),
                new EndScoreCommand(jointSubsystem, extensionSubsystem)),

            new ConeGamepieceCommand(nt),
            new SetIntakeMode(intakeMode, IntakeModeEnum.ForwardDown),
            new MoveAbsoluteAndRotateCommand(swerveSubsystem, pickPosition_in, 0, 70, 0, 5.0)
        );
    }

    private Trajectory makeTrajectoryToPickPounce() {
        var waypoints = new ArrayList<Vector2D>();
        waypoints.add(startPosition_in);
        waypoints.add(mid1APosition_in);
        waypoints.add(pickPouncePosition_in);
        return MakeTrajectory(waypoints, 80.0, 180.0, 80.0); // higher than 90 seems to push the cone
    }

    public Vector2D getStartPosition_in() {
        return startPosition_in;
    }

    public Vector2D getPickPouncePosition_in() {
        return pickPouncePosition_in;
    }

    public double getManualOffsetY_in() {
        return manualOffsetY_in;
    }
}
