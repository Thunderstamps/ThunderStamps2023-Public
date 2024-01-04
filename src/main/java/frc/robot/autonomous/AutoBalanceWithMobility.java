package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.helpers.*;
import frc.robot.communications.*;
import frc.robot.communications.gamepieces.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;

public class AutoBalanceWithMobility extends SequentialCommandGroup {
                
    private static final double startX_in = 56.0;
    private static final double startY_in = 0.0;

    private static final double safetyX_in = startX_in + 104.0; // far enough that it should tip, but without falling off the other side
    private static final double safetyY_in = startY_in;
    private static final Vector2D safetyPosition_in = Vector2D.FromXY(safetyX_in, safetyY_in);

    private static final double chargePounceX_in = 235.0; // outside the community to get the mobility points
    private static final double chargePounceY_in = startY_in;
    private static final Vector2D chargePouncePosition_in = Vector2D.FromXY(chargePounceX_in, chargePounceY_in);

    private static final double communityX_in = 90.0;
    private static final double communityY_in = chargePounceY_in;
    private static final Vector2D communityPosition_in = Vector2D.FromXY(communityX_in, communityY_in);

    private static final double climbHeading_rad = Math.toRadians(90); // just have to be sideways
    
    public AutoBalanceWithMobility(
            NetworkTableComms nt,
            SwerveSubsystem swerveSubsystem,
            GripperSubsystem gripperSubsystem,
            JointSubsystem jointSubsystem,
            ExtensionSubsystem extensionSubsystem,
            IGyro gyro) {

        var speed_in_s = 50.0;

        var continueAfterTilted_in = 22.0;

        var waypoint1_in = Vector2D.FromXY(startX_in + 12.0, 0);  // get the nose out of the grid and turn
        var waypoint1_rad = Math.toRadians(0);
        var waypoint2_in = Vector2D.FromXY(startX_in + 30.0, 0);  // get the nose out of the grid and turn
        var waypoint2_rad = climbHeading_rad;

        this.addCommands(
            new SetRobotFieldPositionCommand(swerveSubsystem, startX_in, startY_in),

            new CubeGamepieceCommand(nt),
            new ParallelDeadlineGroup(
                new BeginScoreHighCommand(nt, gripperSubsystem, jointSubsystem, extensionSubsystem, 0),
                new StopCommand(swerveSubsystem)),

            new ParallelCommandGroup(
                new SequentialCommandGroup(

                    // get the nose out of the grid
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, waypoint1_in, waypoint1_rad, speed_in_s, speed_in_s, 2.0),
                    // rotate
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, waypoint2_in, waypoint2_rad, speed_in_s/2, speed_in_s/2, 2.0),

                    // turn and go up the charge station but make sure it tips and then balances again (avoid falling off the other side)
                    new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                            new WaitForTip(gyro, 11.0, 500),
                            new WaitForBalance(gyro, 10.0)
                        ), 
                        new SequentialCommandGroup(
                            new MoveAbsoluteAndRotateCommand(swerveSubsystem, safetyPosition_in, climbHeading_rad, speed_in_s, speed_in_s, 2.0),
                            new StopCommand(swerveSubsystem)
                        )
                    ),

                    // go over the charge station to get the mobility points
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, chargePouncePosition_in, climbHeading_rad, speed_in_s, speed_in_s, 10.0),

                    // Drive onto the charge station (fast speed) until we detect that the robot is tipped past a certain angle, indicating we're fully on
                    new ParallelDeadlineGroup(
                        new WaitForTip(gyro, 9.0, 500),
                        new MoveAbsoluteAndRotateCommand(swerveSubsystem, communityPosition_in, climbHeading_rad, speed_in_s, 0.0, 0.5)
                    ),

                    // Continue driving the same direction (fast speed) just to save time
                    new MoveRelativeCommand(swerveSubsystem, -continueAfterTilted_in, 0),

                    // Keeps trying to balance until we run out of time
                    new DriveUphillCommand(swerveSubsystem)
                ),
                new EndScoreCommand(jointSubsystem, extensionSubsystem)
            ),

            new StopCommand(swerveSubsystem)
        );
    }
}
