package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.helpers.*;
import frc.robot.communications.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.intake.IntakeMode;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;

public class AutoLeftScore1PickAndBalance extends AutoBase {

    private static final double chargePounceX_in = 220.0;
    private static final double chargePounceY_in = 130.0 - 12.0; // -12 to get closer to center
    private static final Vector2D chargePouncePosition_in = Vector2D.FromXY(chargePounceX_in, chargePounceY_in);

    private static final double communityX_in = 90.0;
    private static final double communityY_in = chargePounceY_in;
    private static final Vector2D communityPosition_in = Vector2D.FromXY(communityX_in, communityY_in);

    private static final double climbHeading_rad = Math.toRadians(-90); // want the nose wheel nearest the left edge of the charge station
    
    
    public AutoLeftScore1PickAndBalance(
            NetworkTableComms nt,
            SwerveSubsystem swerveSubsystem,
            GripperSubsystem gripperSubsystem,
            JointSubsystem jointSubsystem,
            ExtensionSubsystem extensionSubsystem,
            IntakeMode intakeMode,
            IGyro gyro) {

        var speed_in_s = 60.0;

        var continueAfterTilted_in = 22.0;

        var autoLeftScoreConeAndPickCommand = new AutoLeftScoreConeAndPick(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode);

        addCommands(
            autoLeftScoreConeAndPickCommand,

            // Grip the piece while we climb onto the charge station
            new ParallelCommandGroup(
                new AutoGripCone(intakeMode, gripperSubsystem),

                new SequentialCommandGroup(

                    // Move on a diagonal to get lined up for the charge station
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, chargePouncePosition_in, climbHeading_rad, 150.0, speed_in_s, 10.0),

                    // Drive onto the charge station (fast speed) until we detect that the robot is tipped past a certain angle, indicating we're fully on
                    new ParallelDeadlineGroup(
                        new WaitForTip(gyro, 9.0, 500),
                        new MoveAbsoluteAndRotateCommand(swerveSubsystem, communityPosition_in, climbHeading_rad, speed_in_s, 0.0, 0.5)
                    ),

                    // Continue driving the same direction (fast speed) just to save time
                    new MoveRelativeCommand(swerveSubsystem, -continueAfterTilted_in, 0),

                    // Keeps trying to balance until we run out of time
                    new DriveUphillCommand(swerveSubsystem)
                )),
            new StopCommand(swerveSubsystem)
        );
    }
}
