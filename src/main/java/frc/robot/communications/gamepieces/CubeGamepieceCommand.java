package frc.robot.communications.gamepieces;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.*;

public class CubeGamepieceCommand extends CommandBase {
    
    private NetworkTableComms nt;

    public CubeGamepieceCommand(
            NetworkTableComms nt) {
                this.nt = nt;
        this.setName("Cube");
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        this.nt.setGamepieceCube();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
