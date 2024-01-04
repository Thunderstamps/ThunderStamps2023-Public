package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.helpers.*;
import frc.robot.communications.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.intake.IntakeMode;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;

public class AutoRightScore1Pick extends AutoBase {
    
    public AutoRightScore1Pick(
            NetworkTableComms nt,
            SwerveSubsystem swerveSubsystem,
            GripperSubsystem gripperSubsystem,
            JointSubsystem jointSubsystem,
            ExtensionSubsystem extensionSubsystem,
            IntakeMode intakeMode) {

        var autoRightScoreConeAndPickCommand = new AutoRightScoreConeAndPick(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode);

        var pickPouncePosition_in = autoRightScoreConeAndPickCommand.getPickPouncePosition_in();

        this.addCommands(
            autoRightScoreConeAndPickCommand,

            // Grip the piece while heading back towards community.
            new ParallelCommandGroup(
                new AutoGripCone(intakeMode, gripperSubsystem),
                new SequentialCommandGroup(
                    new MoveAbsoluteAndRotateCommand(swerveSubsystem, pickPouncePosition_in, 0, 150, 0, 10.0),
                    new StopCommand(swerveSubsystem)
                )
            )
        );
    }
}
