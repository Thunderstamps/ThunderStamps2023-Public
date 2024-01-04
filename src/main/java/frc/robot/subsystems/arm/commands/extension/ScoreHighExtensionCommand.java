package frc.robot.subsystems.arm.commands.extension;

import frc.robot.subsystems.arm.ExtensionSubsystem;

public class ScoreHighExtensionCommand extends ExtensionPositionCommand {

    private ExtensionSubsystem extensionSubsystem;
    private double targetPosition_rot;

    public ScoreHighExtensionCommand(
            ExtensionSubsystem extensionSubsystem) {
        super(extensionSubsystem);
        this.extensionSubsystem = extensionSubsystem;
        
    }

    @Override
    public void initialize() {
        this.targetPosition_rot = this.extensionSubsystem.MoveToScoreHigh();
    }

    @Override
    public boolean isFinished() {
        return this.extensionSubsystem.isAtPosition(this.targetPosition_rot);
    }
    
}
