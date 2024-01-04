package frc.robot.subsystems.arm.commands;

import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.commands.extension.*;
import frc.robot.subsystems.arm.commands.joint.*;

public class ArmToScoreHighCommand extends ArmMotionCommandBase {

    public ArmToScoreHighCommand(
        JointSubsystem jointSubsystem, 
        ExtensionSubsystem extensionSubsystem) {
        super(jointSubsystem, extensionSubsystem);
        
        this.addCommands(
            new RetractExtensionCommand(extensionSubsystem),
            new ScoreHighJointCommand(jointSubsystem),
            new ScoreHighExtensionCommand(extensionSubsystem)
        );
    }
    
}
