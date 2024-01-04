package frc.robot.subsystems.arm.commands.extension;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ExtensionSubsystem;

public class ExtensionPositionCommand extends CommandBase {

    public ExtensionPositionCommand(
            ExtensionSubsystem extensionSubsystem) {

        this.addRequirements(extensionSubsystem);
    }

}
