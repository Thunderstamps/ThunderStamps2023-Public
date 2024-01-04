package frc.robot.communications.gamepieces;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.*;

public class ConeGamepieceCommand extends CommandBase {
    
    private NetworkTableComms nt;

    public ConeGamepieceCommand(
            NetworkTableComms nt) {
                this.nt = nt;
        this.setName("Cone");
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        this.nt.setGamepieceCone();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
