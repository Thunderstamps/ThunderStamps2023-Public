package frc.robot.subsystems.arm.commands.extension;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.*;

public class DefaultExtensionCommand extends CommandBase {

    public DefaultExtensionCommand(
            ExtensionSubsystem extensionSubsystem) {

        this.addRequirements(extensionSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }
}
