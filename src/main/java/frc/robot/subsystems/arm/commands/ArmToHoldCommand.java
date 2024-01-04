package frc.robot.subsystems.arm.commands;

import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.commands.extension.*;
import frc.robot.subsystems.arm.commands.joint.*;

public class ArmToHoldCommand extends ArmMotionCommandBase {

    public ArmToHoldCommand(
        GripperSubsystem gripperSubsystem,
        JointSubsystem jointSubsystem, 
        ExtensionSubsystem extensionSubsystem) {
        super(jointSubsystem, extensionSubsystem);
        
        this.addCommands(
            new RetractExtensionCommand(extensionSubsystem),
            new PickJointCommand(jointSubsystem)
        );
    }
}
