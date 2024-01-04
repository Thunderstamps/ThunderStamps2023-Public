package frc.robot.autonomous.helpers;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.commands.extension.*;
import frc.robot.subsystems.arm.commands.joint.*;

// Retracts the extension (if not already retracted), moves joint to pick, extends extension to pick
public class EndScoreCommand extends SequentialCommandGroup {
    public EndScoreCommand(
            JointSubsystem jointSubsystem,
            ExtensionSubsystem extensionSubsystem) {
        this.addCommands(
            new RetractExtensionCommand(extensionSubsystem),
            new PickJointCommand(jointSubsystem),
            new PickExtensionCommand(extensionSubsystem)
        );
    }
}
