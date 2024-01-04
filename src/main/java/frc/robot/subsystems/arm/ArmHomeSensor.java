package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DigitalInput;

public class ArmHomeSensor {
    private final DigitalInput sensor = new DigitalInput(0);
    
    public boolean atHome() {
        return this.sensor.get();
    }
}
