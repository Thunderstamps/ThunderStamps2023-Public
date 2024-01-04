package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.helpers.*;
import frc.robot.communications.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;

public class AutoBalance extends SequentialCommandGroup {
    
    public AutoBalance(
            NetworkTableComms nt,
            SwerveSubsystem swerveSubsystem,
            GripperSubsystem gripperSubsystem,
            JointSubsystem jointSubsystem,
            ExtensionSubsystem extensionSubsystem,
            IGyro gyro) {

        var speed_in_s = 36.0;
        var creepSpeed_in_s = 10.0;
        var forwardAfterTilted_in = 19.0;
        var backup_in = 0.0;

        var waypoint1_in = Vector2D.FromXY(12, 0);
        var waypoint1_rad = Math.toRadians(0);

        var waypoint2_in = Vector2D.FromXY(24, 0);
        var waypoint2_rad = Math.toRadians(90);

        var waypoint3_in = Vector2D.FromXY(180, 0);
        var waypoint3_rad = Math.toRadians(90);

        this.addCommands(
            new SetRobotFieldPositionCommand(swerveSubsystem, 0, 0),

            new ParallelDeadlineGroup(
                new BeginScoreHighCommand(nt, gripperSubsystem, jointSubsystem, extensionSubsystem, 0),
                new StopCommand(swerveSubsystem)),

            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, waypoint1_in, waypoint1_rad, speed_in_s, speed_in_s, 2.0),
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, waypoint2_in, waypoint2_rad, speed_in_s, speed_in_s, 3.0),
                    new ParallelDeadlineGroup(
                        new WaitForTip(gyro, 11.0, 500),
                        new MoveAbsoluteAndRotateCommand(swerveSubsystem, waypoint3_in, waypoint3_rad, speed_in_s, 0.0, 0.5)
                    ),
                    new MoveRelativeCommand(swerveSubsystem, forwardAfterTilted_in, 0)
                ),
                new EndScoreCommand(jointSubsystem, extensionSubsystem)
            ),
            
            new ParallelRaceGroup(
                new MoveAbsoluteAndRotateCommand(swerveSubsystem, waypoint3_in, waypoint3_rad, creepSpeed_in_s, 0.0, 0.5),
                new WaitForBalance(gyro, 10.0)
            ),
            new MoveRelativeCommand(swerveSubsystem, -backup_in, 0, 0.5),
            new StopCommand(swerveSubsystem)
        );
    }
}
