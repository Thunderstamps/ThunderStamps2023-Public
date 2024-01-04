package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;

public class MoveRelativeCommand extends CommandBase {

    private static final double DEFAULT_TOLERANCE_IN = 2.0;
    private final SwerveSubsystem swerveSubsystem;
    private final double relativeX_in;
    private final double relativeY_in;
    private Vector2D targetPosition;
    private Vector2D distanceToGo;
    private double tolerance_in;
    
    public MoveRelativeCommand(
            SwerveSubsystem swerveSubsystem,
            double relativeX_in,
            double relativeY_in) {
        this(swerveSubsystem, relativeX_in, relativeY_in, DEFAULT_TOLERANCE_IN);

    }
    
    public MoveRelativeCommand(
        SwerveSubsystem swerveSubsystem,
        double relativeX_in,
        double relativeY_in,
        double tolerance_in) {
        this.swerveSubsystem = swerveSubsystem;
        this.tolerance_in = tolerance_in;
        this.addRequirements(swerveSubsystem);
        this.relativeX_in = relativeX_in;
        this.relativeY_in = relativeY_in;
    }

    @Override
    public void initialize() {
        var initialPosition = this.swerveSubsystem.getPose().getTranslation();
        this.targetPosition = Vector2D.FromXY(
            initialPosition.getX() + this.relativeX_in,
            initialPosition.getY() + this.relativeY_in
        );
        this.updateDistanceToGo();
    }

    @Override
    public void execute() {
        this.updateDistanceToGo();
        var fieldOrientedTranslationCommand_in_s = Vector2D.FromPolar(
            this.distanceToGo.getMagnitude() * 4.0, 
            this.distanceToGo.getAngleRadians());
        this.swerveSubsystem.executeAutonomousControl(
            fieldOrientedTranslationCommand_in_s, 
            Vector2D.FromXY(0, 0),
            0.0);
    }
    //x-54,75
    @Override
    public boolean isFinished() {
        return this.distanceToGo.getMagnitude() <= this.tolerance_in;
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveSubsystem.executeAutonomousControl(
            Vector2D.FromXY(0, 0),
            Vector2D.FromXY(0, 0),
            0.0);
    }

    private void updateDistanceToGo() {
        var position = this.swerveSubsystem.getPose().getTranslation();
        this.distanceToGo = Vector2D.FromXY(
            this.targetPosition.getX() - position.getX(),
            this.targetPosition.getY() - position.getY()
        );
    }
}