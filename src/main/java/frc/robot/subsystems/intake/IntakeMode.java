package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Joystick;

public class IntakeMode {
    private final Joystick joystick;
    private IntakeModeEnum memory = IntakeModeEnum.Off;

    public IntakeMode(Joystick joystick) {
        this.joystick = joystick;

    }

    public IntakeModeEnum getMode() {
        return this.memory;
    }

    public void setMode(IntakeModeEnum newMode) {
        System.out.print("Intake mode set to " + newMode.toString());
        System.out.println();
        this.memory = newMode;
    }

    public double getThrottle() {
        var throttle = (1.0 + this.joystick.getThrottle() * -1) / 2.0;
        return throttle;
    }

    public boolean isForwardUp() {
        return this.memory == IntakeModeEnum.ForwardUp;
    }
}
