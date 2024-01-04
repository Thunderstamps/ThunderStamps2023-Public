package frc.robot.autonomous.helpers;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;

public class TestCommand extends SequentialCommandGroup {
    
    public TestCommand(SwerveSubsystem swerveSubsystem) {

        this.addCommands(
            new SetRobotFieldPositionCommand(swerveSubsystem, 0, 0),
            new MoveXDirectionAtSpeedCommand(swerveSubsystem, -10.0),
            new StopCommand(swerveSubsystem)
        );
    }
}
