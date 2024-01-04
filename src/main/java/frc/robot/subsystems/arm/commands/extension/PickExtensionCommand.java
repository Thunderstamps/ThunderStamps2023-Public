package frc.robot.subsystems.arm.commands.extension;

import frc.robot.subsystems.arm.ExtensionSubsystem;

public class PickExtensionCommand extends ExtensionPositionCommand {
    
    private ExtensionSubsystem extensionSubsystem;

    public PickExtensionCommand(
            ExtensionSubsystem extensionSubsystem){
        super(extensionSubsystem);
        this.extensionSubsystem = extensionSubsystem;
    }

    @Override
    public void initialize() {
        this.extensionSubsystem.MoveToPick();
    }

    @Override
    public boolean isFinished() {
        return this.extensionSubsystem.isAtPick();
    }
}
