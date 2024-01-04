package frc.robot.communications.scorelocations;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.*;

public class HighScoreLocationCommand extends CommandBase {
    
    private NetworkTableComms nt;

    public HighScoreLocationCommand(
            NetworkTableComms nt) {
                this.nt = nt;
        this.setName("Score Location High");
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        this.nt.setScoreLocationHigh();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
