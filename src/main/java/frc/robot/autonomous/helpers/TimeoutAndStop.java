package frc.robot.autonomous.helpers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.commands.StopCommand;

public class TimeoutAndStop extends SequentialCommandGroup {

    public TimeoutAndStop(
        SwerveSubsystem swerveSubsystem, 
        Command command,
        double timeout_sec) {
        
        this.addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(timeout_sec), 
                command),
            new StopCommand(swerveSubsystem)
        );
    }
    
}
