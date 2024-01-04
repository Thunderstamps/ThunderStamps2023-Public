package frc.robot.subsystems.arm.commands;

import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.commands.extension.*;
import frc.robot.subsystems.arm.commands.gripper.ReleaseGripperCommand;
import frc.robot.subsystems.arm.commands.joint.*;

public class ArmToPickCommand extends ArmMotionCommandBase {

    public ArmToPickCommand(
        GripperSubsystem gripperSubsystem,
        JointSubsystem jointSubsystem, 
        ExtensionSubsystem extensionSubsystem) {
        super(jointSubsystem, extensionSubsystem);
        
        this.addCommands(
            new ReleaseGripperCommand(gripperSubsystem),
            new RetractExtensionCommand(extensionSubsystem),
            new PickJointCommand(jointSubsystem),
            new PickExtensionCommand(extensionSubsystem)
        );
    }
    
}
