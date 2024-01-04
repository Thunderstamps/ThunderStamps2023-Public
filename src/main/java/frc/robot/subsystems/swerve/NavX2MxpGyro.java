package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.NavX.AHRS;

public class NavX2MxpGyro extends NavXBase implements IGyro {

    private final AHRS gyro;
    private double scaleFactor;
    private boolean enabled = true;
    private double offsetToZero_rad;

    public NavX2MxpGyro(double scaleFactor) {
        this.gyro = new AHRS(SPI.Port.kMXP, (byte)50);
        this.gyro.enableLogging(true);
        this.scaleFactor = scaleFactor;
    }

    @Override
    protected AHRS getGyro() {
        return this.gyro;
    }

    public double getFieldOrientation_rad() {
        double rawFieldOrientation_rad = gyro.getRotation2d().getRadians() * this.scaleFactor - offsetToZero_rad;
        return SwerveUtil.limitAngleFromNegativePItoPI_rad(rawFieldOrientation_rad);
    }

    public double getRotationRate_rad_s() {
        double rate = Math.toRadians(-gyro.getRate());
        return rate;
    }

    public void resetOffsetToZero_rad() {
        gyro.reset();
        gyro.zeroYaw();
        this.offsetToZero_rad = 0;
    }

    public void adjustOffset_rad(double adjustment_rad) {
        this.offsetToZero_rad += adjustment_rad;
    }

    public boolean getEnabled() {
        return this.enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public double getZAccel_G() {
        return this.gyro.getWorldLinearAccelZ();
    }

    

}