package frc.robot.communications.gamepieces;

import edu.wpi.first.wpilibj.*;

public class LedController {
    private final Solenoid green1 = new Solenoid(2, PneumaticsModuleType.REVPH, 10);
    private final Solenoid red1 = new Solenoid(2, PneumaticsModuleType.REVPH, 11);
    private final Solenoid blue1 = new Solenoid(2, PneumaticsModuleType.REVPH, 12);
    private final Solenoid green2 = new Solenoid(2, PneumaticsModuleType.REVPH, 14);
    private final Solenoid blue2 = new Solenoid(2, PneumaticsModuleType.REVPH, 13);
    private final Solenoid red2 = new Solenoid(2, PneumaticsModuleType.REVPH, 15);

    public LedController() {
        this.SetOff();
    }

    public void SetOff() {
        this.red1.set(false);
        this.green1.set(false);
        this.blue1.set(false);

        this.red2.set(false);
        this.green2.set(false);
        this.blue2.set(false);
    }

    public void SetCone() {
        // yellow
        this.red1.set(true);
        this.green1.set(true);
        this.blue1.set(false);

        this.red2.set(true);
        this.green2.set(true);
        this.blue2.set(false);
    }

    public void SetCube() {
        // purple-ish
        this.red1.set(true);
        this.green1.set(false);
        this.blue1.set(true);

        this.red2.set(true);
        this.green2.set(false);
        this.blue2.set(true);
    }
}
