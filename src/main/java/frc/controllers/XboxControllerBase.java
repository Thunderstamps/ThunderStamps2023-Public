package frc.controllers;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public abstract class XboxControllerBase implements IXboxController {

    private final XboxController xboxController;
    private final PovService povService;

    private double leftStickOffsetX = 0.0;
    private double leftStickOffsetY = 0.0;
    private double rightStickOffsetX = 0.0;
    private double rightStickOffsetY = 0.0;

    private Instant rumbleStart = Instant.now();
    private boolean rumbling = false;

    protected XboxControllerBase(int port) {
        this.xboxController = new XboxController(port);
        this.povService = new PovService(xboxController);
    }

    public PovService getPovService() {
        return this.povService;
    }

    public boolean getXButton() {
        return this.xboxController.getXButton();
    }

    public boolean getBackButtonPressed() {
        return this.xboxController.getBackButtonPressed();
    }

    public boolean getStartButtonPressed() {
        return this.xboxController.getStartButtonPressed();
    }

    public boolean getAButtonPressed() {
        return this.xboxController.getAButtonPressed();
    }

    public boolean getBButtonPressed() {
        return this.xboxController.getBButtonPressed();
    }

    public boolean getXButtonPressed() {
        return this.xboxController.getXButtonPressed();
    }

    public boolean getYButtonPressed() {
        return this.xboxController.getYButtonPressed();
    }

    public boolean getLeftBumperPressed() {
        return this.xboxController.getLeftBumperPressed();
    }

    public boolean getRightBumperPressed() {
        return this.xboxController.getRightBumperPressed();
    }

    public double getLeftTriggerAxis() {
        return this.xboxController.getLeftTriggerAxis();
    }

    public double getRightTriggerAxis() {
        return this.xboxController.getRightTriggerAxis();
    }

    protected void setLeftStickOffsetX(double offset) {
        this.leftStickOffsetX = offset;
    }

    protected void setLeftStickOffsetY(double offset) {
        this.leftStickOffsetY = offset;
    }

    protected void setRightStickOffsetX(double offset) {
        this.rightStickOffsetX = offset;
    }

    protected void setRightStickOffsetY(double offset) {
        this.rightStickOffsetY = offset;
    }

    public double getLeftX() {
        stopRumbling();
        var offset = 0.0;
        offset = this.leftStickOffsetX;
        var result = this.xboxController.getLeftX() + offset;
        return limitRange(result);
    }
    public double getRightX() {
        stopRumbling();
        var offset = 0.0;
        offset = this.rightStickOffsetX;
        var result = this.xboxController.getRightX() + offset;
        return limitRange(result);
    }


    public double getLeftY() {
        stopRumbling();
        var offset = 0.0;
        offset = this.leftStickOffsetY;
        var result = this.xboxController.getLeftY() + offset;
        return limitRange(result);
    }

    public double getRightY() {
        stopRumbling();
        var offset = 0.0;
        offset = this.rightStickOffsetY;
        var result = this.xboxController.getRightY() + offset;
        return limitRange(result);
    }

    private double limitRange(double input) {
        var result = input;
        if(result > 1.0) {
            result = 1.0;
        }
        if(result < -1.0) {
            result = -1.0;
        }
        return result;
    }

    public void printRawAnalogs() {
        printAnalog("LX", this.xboxController.getLeftX());
        printAnalog("LY", this.xboxController.getLeftY());
        printAnalog("RX", this.xboxController.getRightX());
        printAnalog("RY", this.xboxController.getRightY());
        System.out.println();
    }

    private void printAnalog(String name, double value) {
        System.out.print(name);
        System.out.print(":");
        System.out.printf("%.2f", value);
        System.out.print(", ");
    }

    public void rumble() {
        this.rumbleStart = Instant.now();
        this.rumbling = true;
        this.xboxController.setRumble(RumbleType.kLeftRumble, 1.0);
    }

    private void stopRumbling() {
        if(this.rumbling) {
            var duration = Duration.between(this.rumbleStart, Instant.now());
            if(duration.toMillis() >= 500) {
                this.rumbling = false;
                this.xboxController.setRumble(RumbleType.kLeftRumble, 0.0);
                this.xboxController.setRumble(RumbleType.kRightRumble, 0.0);
            }
        }

    }

    @Override
    public XboxController getXboxController() {
        return this.xboxController;
    }
}