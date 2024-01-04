package frc.robot.subsystems.swerve;

import frc.robot.NavX.*;

public abstract class NavXBase {


    protected abstract AHRS getGyro();
    
    public double getTiltAngle_deg() {
        var pitch_deg = this.getGyro().getPitch();
        return Math.abs(pitch_deg);
    }

    public double getRoll_deg() {
        var roll_deg = this.getGyro().getRoll();
        return roll_deg;
    }

    public double getPitch_deg() {
        var pitch_deg = this.getGyro().getPitch();
        return pitch_deg;
    }

    public boolean isTilted() {
        return this.getTiltAngle_deg() > 11.0;
    }

    public boolean isNotTilted() {
        return this.getTiltAngle_deg() < 10.0;
    }
}
