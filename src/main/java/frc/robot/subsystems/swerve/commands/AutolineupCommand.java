package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;

public class AutolineupCommand extends CommandBase  {
    
    private SwerveSubsystem swerveSubsystem;

    public AutolineupCommand(
        SwerveSubsystem swerveSubsystem) {
            this.swerveSubsystem = swerveSubsystem;

        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        this.swerveSubsystem.executeAutoLineup();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
