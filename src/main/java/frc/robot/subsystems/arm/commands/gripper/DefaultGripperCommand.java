package frc.robot.subsystems.arm.commands.gripper;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.GripperSubsystem;

public class DefaultGripperCommand extends CommandBase {

    public DefaultGripperCommand(
            GripperSubsystem gripperSubsystem) {

        this.addRequirements(gripperSubsystem);
    }
}
