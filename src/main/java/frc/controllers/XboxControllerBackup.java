package frc.controllers;

public class XboxControllerBackup extends XboxControllerBase {

    public XboxControllerBackup(int port) {
        super(port);
        setLeftStickOffsetX(-0.05);
        setLeftStickOffsetY(0.05);
        setRightStickOffsetX(-0.05);
        setRightStickOffsetY(0.07);
    }
}