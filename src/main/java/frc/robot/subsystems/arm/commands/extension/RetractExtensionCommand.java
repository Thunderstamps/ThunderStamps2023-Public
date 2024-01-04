package frc.robot.subsystems.arm.commands.extension;

import frc.robot.subsystems.arm.ExtensionSubsystem;

public class RetractExtensionCommand extends ExtensionPositionCommand {
    
    private ExtensionSubsystem extensionSubsystem;

    public RetractExtensionCommand(
            ExtensionSubsystem extensionSubsystem) {
        super(extensionSubsystem);
        this.extensionSubsystem = extensionSubsystem;
    }

    @Override
    public void initialize() {
        this.extensionSubsystem.MoveToRetract();
    }

    @Override
    public boolean isFinished() {
        return this.extensionSubsystem.isRetracted();
    }
}
