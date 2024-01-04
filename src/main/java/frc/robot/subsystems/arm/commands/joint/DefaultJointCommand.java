package frc.robot.subsystems.arm.commands.joint;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.*;

public class DefaultJointCommand extends CommandBase {
    
    public DefaultJointCommand(
            JointSubsystem jointSubsystem) {

        this.addRequirements(jointSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }
}
