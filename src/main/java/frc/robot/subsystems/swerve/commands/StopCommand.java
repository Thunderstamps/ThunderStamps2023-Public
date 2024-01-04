package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;

public class StopCommand extends CommandBase {
    
    private final SwerveSubsystem swerveSubsystem;

    public StopCommand(
        SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing Stop Command");
    }
    
    @Override
    public void execute() {
        this.swerveSubsystem.executeAutonomousControl(
            Vector2D.FromXY(0, 0),
            Vector2D.FromXY(0, 0),
            0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveSubsystem.executeAutonomousControl(
            Vector2D.FromXY(0, 0),
            Vector2D.FromXY(0, 0),
            0.0);
    }
}
