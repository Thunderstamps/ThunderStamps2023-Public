package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveScoringModeOffCommand extends CommandBase {
    private SwerveSubsystem swerveSubsystem;

    public SwerveScoringModeOffCommand(
        SwerveSubsystem swerveSubsystem) {
            this.swerveSubsystem = swerveSubsystem;

    }

    @Override
    public void initialize() {
        this.swerveSubsystem.setRotationP(Robot.ROTATION_P);
        this.swerveSubsystem.setMaxCurrentLimit();
        var fieldOrientation_rad = swerveSubsystem.getFieldOrientation_rad();
        this.swerveSubsystem.setTargetFieldOrientation_rad(fieldOrientation_rad);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
