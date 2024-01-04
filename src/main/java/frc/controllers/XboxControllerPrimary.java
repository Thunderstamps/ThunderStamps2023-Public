package frc.controllers;

public class XboxControllerPrimary extends XboxControllerBase {

    public XboxControllerPrimary(int port) {
        super(port);
        setLeftStickOffsetX(-0.08);
        setLeftStickOffsetY(-0.09);
        setRightStickOffsetX(-0.01);
        setRightStickOffsetY(-0.03);
    }
}