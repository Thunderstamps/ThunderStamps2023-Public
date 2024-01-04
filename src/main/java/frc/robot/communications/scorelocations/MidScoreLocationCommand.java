package frc.robot.communications.scorelocations;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.*;

public class MidScoreLocationCommand extends CommandBase {
    
    private NetworkTableComms nt;

    public MidScoreLocationCommand(
            NetworkTableComms nt) {
                this.nt = nt;
        this.setName("Score Location Mid");
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        this.nt.setScoreLocationMid();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
